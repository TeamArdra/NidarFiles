"""Telemetry from the Jetson over the MicroLR900 radio -- the GCS's only
view of the drone (there is no Wi-Fi link).

RadioLink (app/radio_link.py) owns the serial port and hands every MAVLink
message from the Jetson to on_message(). This class turns them back into
the same ROS-shaped dicts RosBridgeClient caches, so every route in
app/main.py reads them through the same latest()/age_s()/
statustext_history() interface and can't tell the two apart.

What arrives (wire format: app/radio_protocol.py and onboard-autonomy's
telem_command_codec.py / radio_telemetry.py):

    Jetson 1/191  HEARTBEAT            -> link up, mission state, /gcs/heartbeat age
                  STATUSTEXT           -> hover mission `detail`
                  NAMED_VALUE_FLOAT    -> hover progress values
    relay  1/1    HEARTBEAT            -> /mavros/state
                  SYS_STATUS           -> /mavros/battery
                  LOCAL_POSITION_NED   -> /mavros/local_position/pose (+ velocity_local)
                  ATTITUDE_QUATERNION  -> /mavros/imu/data orientation
                  STATUSTEXT           -> /mavros/statustext/recv history

Nothing is reported that didn't just arrive: each value is dropped once
it's older than its stale limit, and the FCU counts as connected only
while its relayed HEARTBEAT keeps arriving. This client can't publish
anything -- START/ABORT go through RadioLink.send_command().
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Any, Callable, Optional

from .radio_protocol import (
    ARDUCOPTER_MODE_NAMES,
    ARENA_EDGES_PER_PART,
    ARENA_GRID_TYPE,
    FCU_RELAY_COMPONENT_ID,
    HOVER_VALUE_KEYS,
    JETSON_COMPONENT_ID,
    JETSON_SYSTEM_ID,
    MAV_MODE_FLAG_GUIDED_ENABLED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MISSION_NAMES_BY_CODE,
    MAP_ROWS_TYPE,
    MISSION_STATE_NAMES,
    PARAM_TEXT_PREFIX,
    SLAM_POSE_TYPE,
    STATUSTEXT_CHUNK_LEN,
)
from .ros_client import (
    ARENA_GRID_TOPIC,
    BATTERY_TOPIC,
    FCU_STATE_TOPIC,
    FLIGHT_TEST_STATUS_TOPIC,
    HEARTBEAT_TOPIC,
    IMU_TOPIC,
    LIDAR_TOPIC,
    MAP_TOPIC,
    POSE_TOPIC,
    SLAM_POSE_TOPIC,
    VALID_COMMANDS,
    VALID_SIMULATION_COMMANDS,
    VELOCITY_TOPIC,
)

JETSON_LINK_TIMEOUT_S = 3.0
FCU_STALE_S = 3.0
BATTERY_STALE_S = 5.0
POSE_STALE_S = 3.0
ATTITUDE_STALE_S = 3.0
HOVER_VALUES_STALE_S = 3.0
# The Jetson resends the mission detail every 5 s while there is one.
MISSION_DETAIL_STALE_S = 12.0
LIDAR_STALE_S = 5.0
SLAM_POSE_STALE_S = 3.0
# No map packet for this long, then a new geometry = a new SLAM map (Jetson
# restarted): start empty instead of shifting the old cells into it.
MAP_RESTART_S = 15.0
# Map packets (2 Hz) keep coming while Cartographer runs.
MAP_STALE_S = 10.0

# The Components panel: (store key, name on the GCS, fresh for this long).
# A part counts as connected while its data keeps arriving over the radio.
COMPONENTS = (
    ("jetson", "Radio link (GCS ↔ Jetson)", JETSON_LINK_TIMEOUT_S),
    ("fcu", "Pixhawk (flight controller)", FCU_STALE_S),
    ("battery", "Battery monitor", BATTERY_STALE_S),
    ("position", "Position estimate (optical flow + rangefinder)", POSE_STALE_S),
    ("orientation", "IMU / attitude", ATTITUDE_STALE_S),
    ("lidar", "LiDAR (RPLIDAR A2)", LIDAR_STALE_S),
    ("map", "SLAM map (Cartographer)", MAP_STALE_S),
)
# OccupancyGrid values the Map panel already draws
_GRID_VALUE = {0: -1, 1: 0, 2: 100}
_MAP_HEADER = struct.Struct("<HhhBBBB")
_POSE = struct.Struct("<hhh")
_ARENA_HEADER = struct.Struct("<BBBhHHhhbbBB")
_UNKNOWN_CM = 0xFFFF
STATUSTEXT_HISTORY = 20
_PARTIAL_TEXT_TIMEOUT_S = 5.0

_NO_PUBLISH = "telemetry comes over the command radio -- there is no ROS link to publish on"

def _rounded(values: dict, digits: int) -> dict:
    """Values arrive as float32 -- drop the noise digits (0.49000000953...)."""
    return {k: round(v, digits) for k, v in values.items()}


class TextAssembler:
    """MAVLink2 STATUSTEXT chunking: id 0 is a whole text; otherwise chunks
    share an id and the one shorter than 50 chars is the last. add()
    returns the full text once complete, else None. Not thread-safe: the
    caller holds its own lock."""

    def __init__(self) -> None:
        self._partial: dict[tuple, dict] = {}

    def add(self, source: tuple, msg, now: float) -> Optional[str]:
        text_id = getattr(msg, "id", 0) or 0
        if text_id == 0:
            return msg.text
        for key in [k for k, p in self._partial.items() if now - p["at"] > _PARTIAL_TEXT_TIMEOUT_S]:
            del self._partial[key]
        key = (source, text_id)
        partial = self._partial.setdefault(key, {"at": now, "chunks": {}})
        partial["chunks"][msg.chunk_seq] = msg.text
        if len(msg.text) >= STATUSTEXT_CHUNK_LEN:
            return None
        del self._partial[key]
        chunks = partial["chunks"]
        if sorted(chunks) != list(range(len(chunks))):
            return None  # a chunk was lost
        return "".join(chunks[i] for i in range(len(chunks)))


# How mission lines are labelled in the messages list.
MISSION_DISPLAY_NAMES = {"hover": "Hover", "motor_test": "Motor Test"}
_MISSION_WARNING_WORDS = ("fail", "reject", "refused", "abort", "lost", "not ", "landing", "kill")

_JETSON = (JETSON_SYSTEM_ID, JETSON_COMPONENT_ID)
_FCU = (JETSON_SYSTEM_ID, FCU_RELAY_COMPONENT_ID)


class RadioTelemetryClient:
    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._values: dict[str, Any] = {}
        self._at: dict[str, float] = {}
        self._hover: dict[str, tuple[float, float]] = {}  # field -> (value, received at)
        self._statustext_history: list[dict] = []
        self._texts = TextAssembler()
        self._map_meta: Optional[tuple] = None  # (cell_cm, origin_x_cm, origin_y_cm, width, height)
        self._map_cells: list[int] = []
        # ARENA_GRID parts of the grid being received: header -> {part: body}
        self._arena_header: Optional[tuple] = None
        self._arena_parts: dict[int, bytes] = {}
        self._mission_id: Optional[str] = None  # from the Jetson heartbeat

    # -- input (RadioLink reader thread) -------------------------------------------

    def on_message(self, msg) -> None:
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        kind = msg.get_type()
        now = self._clock()
        with self._lock:
            if source == _JETSON:
                if kind == "HEARTBEAT":
                    self._store("jetson", MISSION_STATE_NAMES.get(msg.custom_mode & 0xFF, "unknown"), now)
                    self._mission_id = MISSION_NAMES_BY_CODE.get((msg.custom_mode >> 8) & 0xFF)
                elif kind == "TUNNEL" and msg.payload_type in (MAP_ROWS_TYPE, SLAM_POSE_TYPE, ARENA_GRID_TYPE):
                    payload = bytes(msg.payload[: msg.payload_length])
                    if msg.payload_type == MAP_ROWS_TYPE:
                        self._apply_map_rows(payload, now)
                    elif msg.payload_type == ARENA_GRID_TYPE:
                        self._apply_arena_part(payload, now)
                    elif len(payload) >= _POSE.size:
                        x, y, yaw = _POSE.unpack_from(payload)
                        self._store("slam_pose", {"x": x / 100.0, "y": y / 100.0, "yaw_deg": yaw / 100.0}, now)
                elif kind == "OBSTACLE_DISTANCE":
                    increment = msg.increment_f if msg.increment_f > 0 else float(msg.increment)
                    self._store("lidar", {
                        "angle_offset_deg": float(msg.angle_offset),
                        "increment_deg": increment,
                        "min_cm": msg.min_distance,
                        "max_cm": msg.max_distance,
                        # None = no return in that sector
                        "distances_cm": [None if d == _UNKNOWN_CM else d for d in msg.distances],
                    }, now)
                elif kind == "NAMED_VALUE_FLOAT" and msg.name in HOVER_VALUE_KEYS:
                    self._hover[HOVER_VALUE_KEYS[msg.name]] = (round(msg.value, 2), now)
                elif kind == "STATUSTEXT":
                    text = self._reassemble(source, msg, now)
                    if text is not None and not text.startswith(PARAM_TEXT_PREFIX):
                        if text != self._values.get("mission_detail"):
                            self._add_mission_message(text)
                        self._store("mission_detail", text, now)
            elif source == _FCU:
                if kind == "HEARTBEAT":
                    self._store("fcu", {
                        "armed": bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                        "guided": bool(msg.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED),
                        "mode": ARDUCOPTER_MODE_NAMES.get(msg.custom_mode, "UNKNOWN"),
                        "system_status": msg.system_status,
                    }, now)
                elif kind == "SYS_STATUS":
                    self._store("battery", {
                        "voltage": None if msg.voltage_battery == 0xFFFF else msg.voltage_battery / 1000.0,
                        "current": None if msg.current_battery == -1 else msg.current_battery / 100.0,
                        "percentage": None if msg.battery_remaining < 0 else msg.battery_remaining / 100.0,
                    }, now)
                elif kind == "LOCAL_POSITION_NED":
                    self._store("position", _rounded({"x": msg.x, "y": msg.y, "z": msg.z}, 3), now)
                    velocity = (msg.vx, msg.vy, msg.vz)
                    if all(math.isfinite(v) for v in velocity):
                        self._store("velocity", _rounded(dict(zip("xyz", velocity)), 3), now)
                elif kind == "ATTITUDE_QUATERNION":
                    orientation = {"x": msg.q2, "y": msg.q3, "z": msg.q4, "w": msg.q1}
                    self._store("orientation", _rounded(orientation, 4), now)
                elif kind == "STATUSTEXT":
                    text = self._reassemble(source, msg, now)
                    if text is not None:
                        self._statustext_history.append({"severity": msg.severity, "text": text})
                        del self._statustext_history[:-STATUSTEXT_HISTORY]

    def _add_mission_message(self, text: str) -> None:
        """A new mission status line also goes into the messages list, so
        the operator sees each mission step next to the FCU's messages.
        (The Jetson resends an unchanged line every 5 s; only changes land
        here.)"""
        name = MISSION_DISPLAY_NAMES.get(self._mission_id or "", self._mission_id or "Mission")
        lowered = text.lower()
        warning = any(w in lowered for w in _MISSION_WARNING_WORDS)
        self._statustext_history.append({"severity": 4 if warning else 6, "text": f"{name}: {text}"})
        del self._statustext_history[:-STATUSTEXT_HISTORY]

    def _regrid(self, meta: tuple, now: float) -> list[int]:
        """New map geometry. While mapping, the map grows (new size/origin,
        same cell size): copy the cells we have to their same world position
        in the new grid, so the panel keeps showing the map while the Jetson
        re-sends rows. A different cell size, or a long silence (the Jetson
        was restarted: new SLAM frame), starts an empty map instead."""
        cell_cm, ox, oy, w, h = meta
        cells = [-1] * (w * h)
        old = self._map_meta
        last = self._at.get("map")
        if old is None or old[0] != cell_cm or last is None or now - last > MAP_RESTART_S:
            return cells
        _, oox, ooy, ow, oh = old
        dx = (oox - ox) // cell_cm  # old column 0 is new column dx
        dy = (ooy - oy) // cell_cm
        for r in range(oh):
            nr = r + dy
            if not 0 <= nr < h:
                continue
            for c in range(ow):
                nc = c + dx
                if 0 <= nc < w:
                    cells[nr * w + nc] = self._map_cells[r * ow + c]
        return cells

    def _apply_map_rows(self, payload: bytes, now: float) -> None:
        """Write received rows into the map; a new geometry (size, origin,
        cell size) starts a fresh, all-unknown map."""
        if len(payload) < _MAP_HEADER.size:
            return
        cell_cm, ox, oy, w, h, row0, nrows = _MAP_HEADER.unpack_from(payload)
        body = payload[_MAP_HEADER.size :]
        count = w * nrows
        if w == 0 or h == 0 or row0 + nrows > h or len(body) < (count + 3) // 4:
            return
        meta = (cell_cm, ox, oy, w, h)
        if self._map_meta != meta:
            self._map_cells = self._regrid(meta, now)
            self._map_meta = meta
        base = row0 * w
        for i in range(count):
            self._map_cells[base + i] = _GRID_VALUE.get((body[i >> 2] >> ((i & 3) * 2)) & 0x3, -1)
        self._at["map"] = now

    def _apply_arena_part(self, payload: bytes, now: float) -> None:
        """Collect the parts of one grid (same gen and header); once all have
        arrived it replaces the shown grid. A part of a newer grid drops the
        unfinished one."""
        if len(payload) < _ARENA_HEADER.size:
            return
        f = _ARENA_HEADER.unpack_from(payload)
        part, nparts = f[1], f[2]
        if nparts == 0 or part >= nparts:
            return
        header = (f[0], nparts) + f[3:]
        if header != self._arena_header:
            self._arena_header = header
            self._arena_parts = {}
        self._arena_parts[part] = payload[_ARENA_HEADER.size :]
        if len(self._arena_parts) < nparts:
            return
        _gen, _n, theta_cdeg, ou_mm, ov_mm, eu, ev, col0, row0, cols, rows = header
        count = rows * (cols + 1) + (rows + 1) * cols
        edges: list[int] = []
        for k in range(nparts):
            body = self._arena_parts[k]
            edges.extend((body[i >> 2] >> ((i & 3) * 2)) & 0x3 for i in range(len(body) * 4))
        if len(edges) < count or cols == 0 or rows == 0:
            return
        nv = rows * (cols + 1)
        self._store("arena", {
            "theta_deg": theta_cdeg / 100.0,
            "offset_u": ou_mm / 1000.0,
            "offset_v": ov_mm / 1000.0,
            "entry_u": eu,
            "entry_v": ev,
            "col0": col0,
            "row0": row0,
            "cols": cols,
            "rows": rows,
            "v_edges": [edges[r * (cols + 1) : (r + 1) * (cols + 1)] for r in range(rows)],
            "h_edges": [edges[nv + k * cols : nv + (k + 1) * cols] for k in range(rows + 1)],
        }, now)

    def _store(self, key: str, value: Any, now: float) -> None:
        self._values[key] = value
        self._at[key] = now

    def _reassemble(self, source: tuple, msg, now: float) -> Optional[str]:
        return self._texts.add(source, msg, now)

    # -- ros_client interface ----------------------------------------------------

    def _fresh(self, key: str, limit: float) -> Any:
        at = self._at.get(key)
        if at is None or self._clock() - at >= limit:
            return None
        return self._values.get(key)

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._fresh("jetson", JETSON_LINK_TIMEOUT_S) is not None

    def connect(self) -> None:
        """Nothing to connect: RadioLink owns the serial port."""

    def disconnect(self) -> None:
        """Symmetric with connect()."""

    def latest(self, topic: str) -> Any:
        with self._lock:
            if topic == FCU_STATE_TOPIC:
                return self._fcu_state()
            if topic == BATTERY_TOPIC:
                return self._fresh("battery", BATTERY_STALE_S)
            if topic == POSE_TOPIC:
                position = self._fresh("position", POSE_STALE_S)
                return None if position is None else {"pose": {"position": position}}
            if topic == VELOCITY_TOPIC:
                velocity = self._fresh("velocity", POSE_STALE_S)
                return None if velocity is None else {"twist": {"linear": velocity}}
            if topic == IMU_TOPIC:
                orientation = self._fresh("orientation", ATTITUDE_STALE_S)
                return None if orientation is None else {"orientation": orientation}
            if topic == FLIGHT_TEST_STATUS_TOPIC:
                return self._hover_status()
            if topic == MAP_TOPIC:
                if self._map_meta is None:
                    return None
                cell_cm, ox, oy, w, h = self._map_meta
                return {
                    "info": {
                        "resolution": cell_cm / 100.0,
                        "width": w,
                        "height": h,
                        "origin": {"position": {"x": ox / 100.0, "y": oy / 100.0}},
                    },
                    "data": list(self._map_cells),
                    "age_s": round(self._clock() - self._at["map"], 1),
                }
            if topic == SLAM_POSE_TOPIC:
                return self._fresh("slam_pose", SLAM_POSE_STALE_S)
            if topic == ARENA_GRID_TOPIC:
                # like the map: the last grid stays shown, with its age
                arena = self._values.get("arena")
                return None if arena is None else {**arena, "age_s": round(self._clock() - self._at["arena"], 1)}
            if topic == LIDAR_TOPIC:
                scan = self._fresh("lidar", LIDAR_STALE_S)
                return None if scan is None else {**scan, "age_s": round(self._clock() - self._at["lidar"], 1)}
            return None

    def components(self) -> list[dict]:
        """Every drone part the GCS hears from: "ok" while its data keeps
        arriving, "lost" once it stopped, "missing" if never heard."""
        with self._lock:
            now = self._clock()
            out = []
            for key, name, limit in COMPONENTS:
                at = self._at.get(key)
                if at is None:
                    status, age = "missing", None
                else:
                    age = round(now - at, 1)
                    status = "ok" if now - at < limit else "lost"
                detail = self._component_detail(key) if status == "ok" else None
                out.append({"key": key, "name": name, "status": status, "age_s": age, "detail": detail})
            return out

    def _component_detail(self, key: str) -> Optional[str]:
        value = self._values.get(key)
        if key == "jetson":
            return f"mission {value}"
        if key == "fcu":
            return f"{value['mode']}, {'ARMED' if value['armed'] else 'disarmed'}"
        if key == "battery" and value.get("voltage") is not None:
            return f"{value['voltage']:.2f} V"
        if key == "map" and self._map_meta is not None:
            cell_cm, _, _, w, h = self._map_meta
            return f"{w} x {h} cells @ {cell_cm / 100:g} m"
        return None

    def _fcu_state(self) -> Optional[dict]:
        if "jetson" not in self._at and "fcu" not in self._at:
            return None  # never heard from the Jetson: no data, not "disconnected"
        fcu = self._fresh("fcu", FCU_STALE_S)
        if fcu is None:
            return {"connected": False, "armed": None, "guided": None, "mode": None, "system_status": None}
        return {"connected": True, **fcu}

    def _hover_status(self) -> Optional[dict]:
        state = self._fresh("jetson", JETSON_LINK_TIMEOUT_S)
        if state is None or state == "unknown":
            return None
        now = self._clock()
        values = {k: v for k, (v, at) in self._hover.items() if now - at < HOVER_VALUES_STALE_S}
        fcu = self._fresh("fcu", FCU_STALE_S) or {}
        position = self._fresh("position", POSE_STALE_S)
        return {
            "scenario": self._mission_id or "hover",
            "state": state,
            "detail": self._fresh("mission_detail", MISSION_DETAIL_STALE_S),
            "target_altitude_m": values.get("target_altitude_m"),
            "current_altitude_m": values.get("current_altitude_m"),
            "current_position": None if position is None else [round(position[k], 2) for k in "xyz"],
            "duration_s": values.get("duration_s"),
            "elapsed_hover_s": values.get("elapsed_hover_s"),
            "armed": fcu.get("armed"),
            "flight_mode": fcu.get("mode"),
            "execution_mode": "real",
        }

    def age_s(self, topic: str) -> float | None:
        key = {
            HEARTBEAT_TOPIC: "jetson",
            FCU_STATE_TOPIC: "fcu",
            BATTERY_TOPIC: "battery",
            POSE_TOPIC: "position",
            VELOCITY_TOPIC: "velocity",
            IMU_TOPIC: "orientation",
        }.get(topic)
        with self._lock:
            at = None if key is None else self._at.get(key)
            return None if at is None else self._clock() - at

    def survivors(self) -> list[dict]:
        return []

    def statustext_history(self) -> list[dict]:
        with self._lock:
            return list(self._statustext_history)

    def publish_command(self, command: str) -> None:
        if command not in VALID_COMMANDS:
            raise ValueError(f"invalid command: {command!r}; must be one of {VALID_COMMANDS}")
        raise RuntimeError(_NO_PUBLISH)

    def publish_simulation_command(self, command: str) -> None:
        if command not in VALID_SIMULATION_COMMANDS:
            raise ValueError(
                f"invalid simulation command: {command!r}; must be one of {VALID_SIMULATION_COMMANDS}"
            )
        raise RuntimeError(_NO_PUBLISH)

    def publish_mission_select(self, mission_id: str, scenario_id: str) -> None:
        if not mission_id or not scenario_id:
            raise ValueError("mission_id and scenario_id must be non-empty")
        raise RuntimeError(_NO_PUBLISH)

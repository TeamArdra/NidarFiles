"""What the Jetson sends to the GCS over the MicroLR900 radio as telemetry
-- pure decision logic, kept free of rclpy so it's unit-testable (same
split as radio_command_logic.py vs. radio_command_node.py).

There is no Wi-Fi link: the radio is the only path between the Jetson and
the GCS, so everything the operator sees comes through here. Message
choice and wire format: telem_command_codec.py's module docstring.

radio_command_node feeds the latest ROS values in (update_*), calls
tick() at `rate_hz`, and writes the returned frames to the radio. The
radio's air rate is limited and START/ABORT ACKs share it, so the load is
kept small and bounded (about 250 B/s at 2 Hz):

  every tick      LOCAL_POSITION_NED, ATTITUDE_QUATERNION   (if fresh)
  once a second   FCU HEARTBEAT, SYS_STATUS, hover values   (if fresh)
  per tick, max   STATUSTEXT_PER_TICK status-text frames, queued

Stale inputs are not sent, so the GCS sees them go stale too -- nothing
is repeated as if it were current. The FCU HEARTBEAT is only sent while
MAVROS reports connected=true.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Callable, Deque, List, Optional, Tuple

from .arena_grid import ARENA_GRID_TYPE, ArenaGrid, ArenaGridSender
from .map_grid import SLAM_POSE_TYPE, MAP_ROWS_TYPE, GridMeta, MapSender, encode_pose
from .telem_command_codec import (
    FCU_RELAY_COMPONENT_ID,
    HOVER_VALUE_NAMES,
    JETSON_COMPONENT_ID,
    MAV_SEVERITY_INFO,
    encode_attitude_quaternion,
    encode_fcu_heartbeat,
    encode_local_position,
    encode_named_value_float,
    encode_obstacle_distance,
    encode_tunnel,
    encode_statustext,
    encode_sys_status,
    statustext_chunks,
)

DEFAULT_RATE_HZ = 2.0
SLOW_PERIOD_S = 1.0

FCU_STATE_STALE_S = 3.0
BATTERY_STALE_S = 5.0
POSE_STALE_S = 1.0
VELOCITY_STALE_S = 1.0
ATTITUDE_STALE_S = 1.0
MISSION_STATUS_STALE_S = 3.0

# LiDAR: one OBSTACLE_DISTANCE (~180 bytes) per period, only while scans are fresh.
DEFAULT_LIDAR_RATE_HZ = 1.0
SCAN_STALE_S = 1.0
LIDAR_SECTOR_DEG = 5.0

# SLAM map: one MAP_ROWS TUNNEL (<= ~145 B) per period while /map is fresh;
# the SLAM pose (~25 B) every tick while fresh.
DEFAULT_MAP_RATE_HZ = 2.0
MAP_STALE_S = 5.0
SLAM_POSE_STALE_S = 1.0

# 1 m arena grid (arena_grid.py): one ARENA_GRID TUNNEL (<= ~145 B) per period,
# parts in turn, while the grid is fresh (recomputed every few seconds).
DEFAULT_ARENA_RATE_HZ = 1.0
ARENA_STALE_S = 15.0

STATUSTEXT_PER_TICK = 2
STATUSTEXT_QUEUE = 20
MISSION_DETAIL_RESEND_S = 5.0


class Schedule:
    """A fixed timetable: due every `period` seconds, measured from the
    previous DUE time (not from when tick() happened to run), so the
    average rate is exact. A tick arriving up to 10 % of a period early
    still counts. Comparing against the last send time instead made 1 Hz
    become ~0.77 Hz on the Jetson: a 2 Hz tick 1 ms early was skipped and
    the send slipped half a tick, every time (measured 2026-10-09)."""

    EARLY_FRACTION = 0.1

    def __init__(self, period: float) -> None:
        self.period = period
        self._due: Optional[float] = None

    def due(self, now: float) -> bool:
        if self._due is None or now - self._due > self.period:
            self._due = now  # first use, or after a long pause: restart the timetable here
        if now < self._due - self.EARLY_FRACTION * self.period:
            return False
        self._due += self.period
        return True


class TelemetryRelay:
    def __init__(
        self,
        next_seq: Callable[[], int],
        start_time: float,
        lidar_rate_hz: float = DEFAULT_LIDAR_RATE_HZ,
        map_rate_hz: float = DEFAULT_MAP_RATE_HZ,
        arena_rate_hz: float = DEFAULT_ARENA_RATE_HZ,
    ) -> None:
        self._next_seq = next_seq
        self._start = start_time
        self._lidar_schedule = Schedule(1.0 / lidar_rate_hz) if lidar_rate_hz > 0 else None
        self._scan: Optional[Tuple[List[int], int, int]] = None  # sectors, min_cm, max_cm
        self._scan_at: Optional[float] = None
        self._slow_schedule = Schedule(SLOW_PERIOD_S)
        self._map_schedule = Schedule(1.0 / map_rate_hz) if map_rate_hz > 0 else None
        self._map = MapSender()
        self._map_at: Optional[float] = None
        self._arena_schedule = Schedule(1.0 / arena_rate_hz) if arena_rate_hz > 0 else None
        self._arena = ArenaGridSender()
        self._arena_at: Optional[float] = None
        self._slam_pose: Optional[Tuple[float, float, float]] = None
        self._slam_pose_at: Optional[float] = None

        self._fcu: Optional[Tuple[bool, bool, bool, Optional[str], int]] = None
        self._fcu_at: Optional[float] = None
        self._battery: Optional[Tuple[Optional[float], Optional[float], Optional[float]]] = None
        self._battery_at: Optional[float] = None
        self._position: Optional[Tuple[float, float, float]] = None
        self._position_at: Optional[float] = None
        self._velocity: Optional[Tuple[float, float, float]] = None
        self._velocity_at: Optional[float] = None
        self._orientation: Optional[Tuple[float, float, float, float]] = None
        self._orientation_at: Optional[float] = None
        self._mission: Optional[dict] = None
        self._mission_at: Optional[float] = None

        self._mission_detail: Optional[str] = None
        self._mission_detail_sent_at: Optional[float] = None
        # (severity, text, component id), oldest first
        self._texts: Deque[Tuple[int, str, int]] = deque(maxlen=STATUSTEXT_QUEUE)
        # (severity, chunk, text id, chunk seq, component id) not yet sent
        self._pending_chunks: Deque[Tuple[int, bytes, int, int, int]] = deque()
        self._text_id = 0

    # -- inputs -------------------------------------------------------------------

    def update_fcu_state(
        self, now: float, connected: bool, armed: bool, guided: bool, mode: Optional[str], system_status: int
    ) -> None:
        self._fcu = (bool(connected), bool(armed), bool(guided), mode, int(system_status))
        self._fcu_at = now

    def update_battery(
        self, now: float, voltage_v: Optional[float], current_a: Optional[float], fraction: Optional[float]
    ) -> None:
        self._battery = (voltage_v, current_a, fraction)
        self._battery_at = now

    def update_position(self, now: float, x: float, y: float, z: float) -> None:
        self._position = (x, y, z)
        self._position_at = now

    def update_velocity(self, now: float, x: float, y: float, z: float) -> None:
        self._velocity = (x, y, z)
        self._velocity_at = now

    def update_orientation(self, now: float, w: float, x: float, y: float, z: float) -> None:
        self._orientation = (w, x, y, z)
        self._orientation_at = now

    def update_mission_status(self, now: float, status: dict) -> None:
        self._mission = status
        self._mission_at = now
        detail = status.get("detail")
        if detail and detail != self._mission_detail:
            self._mission_detail = detail
            self._mission_detail_sent_at = None  # send on the next tick

    def update_scan(self, now: float, sectors_cm: List[int], min_cm: int, max_cm: int) -> None:
        self._scan = (list(sectors_cm), int(min_cm), int(max_cm))
        self._scan_at = now

    def update_map(self, now: float, meta: GridMeta, cells: bytearray) -> None:
        self._map.update(meta, cells)
        self._map_at = now

    def update_arena(self, now: float, grid: ArenaGrid) -> None:
        self._arena.update(grid)
        self._arena_at = now

    def update_slam_pose(self, now: float, x_m: float, y_m: float, yaw_rad: float) -> None:
        self._slam_pose = (x_m, y_m, yaw_rad)
        self._slam_pose_at = now

    def add_fcu_statustext(self, severity: int, text: str) -> None:
        if text:
            self._texts.append((int(severity), text, FCU_RELAY_COMPONENT_ID))

    # -- output -------------------------------------------------------------------

    def tick(self, now: float) -> List[bytes]:
        frames: List[bytes] = []
        ms = int((now - self._start) * 1000)

        if self._fresh(self._position_at, now, POSE_STALE_S):
            velocity = self._velocity if self._fresh(self._velocity_at, now, VELOCITY_STALE_S) else None
            frames.append(encode_local_position(ms, self._position, velocity, self._next_seq()))
        if self._fresh(self._orientation_at, now, ATTITUDE_STALE_S):
            frames.append(encode_attitude_quaternion(ms, *self._orientation, self._next_seq()))

        if self._slow_schedule.due(now):
            frames.extend(self._slow(now, ms))

        if (
            self._lidar_schedule is not None
            and self._fresh(self._scan_at, now, SCAN_STALE_S)
            and self._lidar_schedule.due(now)
        ):
            sectors, min_cm, max_cm = self._scan
            frames.append(
                encode_obstacle_distance(
                    int((now - self._start) * 1e6), sectors, min_cm, max_cm, LIDAR_SECTOR_DEG, self._next_seq()
                )
            )

        if self._fresh(self._slam_pose_at, now, SLAM_POSE_STALE_S):
            frames.append(encode_tunnel(SLAM_POSE_TYPE, encode_pose(*self._slam_pose), self._next_seq()))
        if (
            self._map_schedule is not None
            and self._fresh(self._map_at, now, MAP_STALE_S)
            and self._map_schedule.due(now)
        ):
            payload = self._map.next_payload(now)
            if payload is not None:
                frames.append(encode_tunnel(MAP_ROWS_TYPE, payload, self._next_seq()))
        if (
            self._arena_schedule is not None
            and self._fresh(self._arena_at, now, ARENA_STALE_S)
            and self._arena_schedule.due(now)
        ):
            payload = self._arena.next_payload()
            if payload is not None:
                frames.append(encode_tunnel(ARENA_GRID_TYPE, payload, self._next_seq()))

        self._queue_mission_detail(now)
        frames.extend(self._statustext_frames())
        return frames

    def _slow(self, now: float, ms: int) -> List[bytes]:
        frames: List[bytes] = []
        if self._fresh(self._fcu_at, now, FCU_STATE_STALE_S) and self._fcu[0]:
            _, armed, guided, mode, system_status = self._fcu
            frames.append(encode_fcu_heartbeat(armed, guided, mode, system_status, self._next_seq()))
        if self._fresh(self._battery_at, now, BATTERY_STALE_S):
            frames.append(encode_sys_status(*self._battery, self._next_seq()))
        if self._fresh(self._mission_at, now, MISSION_STATUS_STALE_S):
            for key, name in HOVER_VALUE_NAMES.items():
                value = self._mission.get(key)
                if isinstance(value, (int, float)) and math.isfinite(value):
                    frames.append(encode_named_value_float(ms, name, float(value), self._next_seq()))
        return frames

    def _queue_mission_detail(self, now: float) -> None:
        if not self._mission_detail or not self._fresh(self._mission_at, now, MISSION_STATUS_STALE_S):
            return
        if (
            self._mission_detail_sent_at is not None
            and now - self._mission_detail_sent_at < MISSION_DETAIL_RESEND_S
        ):
            return
        if any(c == JETSON_COMPONENT_ID for _, _, c in self._texts):
            return  # previous copy still queued
        self._mission_detail_sent_at = now
        self._texts.append((MAV_SEVERITY_INFO, self._mission_detail, JETSON_COMPONENT_ID))

    def _statustext_frames(self) -> List[bytes]:
        """Up to STATUSTEXT_PER_TICK frames; a long text's chunks continue
        on the next tick, never interleaved with another text."""
        while len(self._pending_chunks) < STATUSTEXT_PER_TICK and self._texts:
            severity, text, compid = self._texts.popleft()
            chunks = statustext_chunks(text)
            text_id = 0
            if len(chunks) > 1:
                self._text_id = self._text_id % 0x7FFF + 1  # 0x8000+ is for parameter replies
                text_id = self._text_id
            for chunk_seq, chunk in enumerate(chunks):
                self._pending_chunks.append((severity, chunk, text_id, chunk_seq, compid))
        frames = []
        for _ in range(min(STATUSTEXT_PER_TICK, len(self._pending_chunks))):
            severity, chunk, text_id, chunk_seq, compid = self._pending_chunks.popleft()
            frames.append(
                encode_statustext(severity, chunk, self._next_seq(), text_id, chunk_seq, compid=compid)
            )
        return frames

    @staticmethod
    def _fresh(at: Optional[float], now: float, limit: float) -> bool:
        return at is not None and now - at < limit

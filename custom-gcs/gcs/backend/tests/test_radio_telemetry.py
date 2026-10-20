"""Tests for app/radio_telemetry.RadioTelemetryClient -- telemetry over the
MicroLR900 radio, built from real MAVLink messages (pymavlink) shaped
exactly as the Jetson's radio_command_node sends them. The RF link itself
still needs the hardware test."""

from __future__ import annotations

import math
import time

from fastapi.testclient import TestClient
from pymavlink.dialects.v20 import common as mavlink

from app.config import Settings
from app.main import create_app
from app.radio_link import RadioLink
from app.radio_telemetry import RadioTelemetryClient

from .fakes import FakeRadioLink
from .test_radio_link import FakeJetson, FakeSerial


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


def _sender(component: int) -> mavlink.MAVLink:
    return mavlink.MAVLink(None, srcSystem=1, srcComponent=component)


JETSON = _sender(191)
FCU = _sender(1)


def _decode(raw: bytes):
    return mavlink.MAVLink(None).parse_char(raw)


def _feed(client: RadioTelemetryClient, *messages) -> None:
    for sender, msg in messages:
        client.on_message(_decode(msg.pack(sender)))


def jetson_heartbeat(state_code: int = 0, mission_code: int = 0):
    return JETSON, JETSON.heartbeat_encode(18, 8, 0, (mission_code << 8) | state_code, 4)


def fcu_heartbeat(armed=False, guided=False, mode=0, system_status=3):
    base = 1 | (128 if armed else 0) | (8 if guided else 0)
    return FCU, FCU.heartbeat_encode(2, 3, base, mode, system_status)


def battery(mv=12345, ca=-321, remaining=87):
    return FCU, FCU.sys_status_encode(0, 0, 0, 0, mv, ca, remaining, 0, 0, 0, 0, 0, 0)


def position(x=1.0, y=2.0, z=0.5, vx=0.1, vy=0.0, vz=-0.2):
    return FCU, FCU.local_position_ned_encode(0, x, y, z, vx, vy, vz)


def attitude(w=1.0, x=0.0, y=0.0, z=0.0):
    return FCU, FCU.attitude_quaternion_encode(0, w, x, y, z, 0, 0, 0)


def statustext(sender, text, severity=4, text_id=0, chunk_seq=0):
    return sender, sender.statustext_encode(severity, text.encode(), text_id, chunk_seq)


def named(name, value):
    return JETSON, JETSON.named_value_float_encode(0, name.encode(), value)


# -- client ---------------------------------------------------------------------


def test_nothing_received_means_no_data_not_disconnected():
    client = RadioTelemetryClient(Clock())
    assert client.is_connected is False
    assert client.latest("/mavros/state") is None
    assert client.latest("/mavros/battery") is None
    assert client.age_s("/gcs/heartbeat") is None


def test_jetson_heartbeat_is_the_link():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    _feed(client, jetson_heartbeat(0))
    assert client.is_connected is True
    clock.t += 2.0
    assert client.age_s("/gcs/heartbeat") == 2.0
    clock.t += 1.5
    assert client.is_connected is False


def test_fcu_state_from_relayed_heartbeat():
    client = RadioTelemetryClient(Clock())
    _feed(client, fcu_heartbeat(armed=True, guided=True, mode=4, system_status=4))
    assert client.latest("/mavros/state") == {
        "connected": True, "armed": True, "guided": True, "mode": "GUIDED", "system_status": 4,
    }


def test_fcu_unknown_mode_number():
    client = RadioTelemetryClient(Clock())
    _feed(client, fcu_heartbeat(mode=0xFFFFFFFF))
    assert client.latest("/mavros/state")["mode"] == "UNKNOWN"


def test_fcu_goes_disconnected_when_relay_heartbeat_stops_but_jetson_is_alive():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    _feed(client, fcu_heartbeat(armed=True))
    clock.t += 4.0
    _feed(client, jetson_heartbeat(0))
    state = client.latest("/mavros/state")
    assert state["connected"] is False and state["armed"] is None


def test_jetson_without_fcu_reports_fcu_disconnected():
    client = RadioTelemetryClient(Clock())
    _feed(client, jetson_heartbeat(0))
    assert client.latest("/mavros/state")["connected"] is False


def test_battery_values_and_unknowns():
    client = RadioTelemetryClient(Clock())
    _feed(client, battery())
    assert client.latest("/mavros/battery") == {"voltage": 12.345, "current": -3.21, "percentage": 0.87}
    _feed(client, battery(mv=0xFFFF, ca=-1, remaining=-1))
    assert client.latest("/mavros/battery") == {"voltage": None, "current": None, "percentage": None}


def test_position_velocity_attitude_in_ros_shapes():
    client = RadioTelemetryClient(Clock())
    _feed(client, position(), attitude(0.7071, 0.0, 0.0, 0.7071))
    pose = client.latest("/mavros/local_position/pose")["pose"]["position"]
    assert (pose["x"], pose["y"], pose["z"]) == (1.0, 2.0, 0.5)
    velocity = client.latest("/mavros/local_position/velocity_local")["twist"]["linear"]
    assert velocity == {"x": 0.1, "y": 0.0, "z": -0.2}
    orientation = client.latest("/mavros/imu/data")["orientation"]
    assert orientation == {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071}


def test_nan_velocity_is_not_reported():
    client = RadioTelemetryClient(Clock())
    _feed(client, position(vx=math.nan, vy=math.nan, vz=math.nan))
    assert client.latest("/mavros/local_position/pose") is not None
    assert client.latest("/mavros/local_position/velocity_local") is None


def test_stale_values_are_dropped_not_repeated():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    _feed(client, position(), attitude(), battery())
    clock.t += 10.0
    assert client.latest("/mavros/local_position/pose") is None
    assert client.latest("/mavros/imu/data") is None
    assert client.latest("/mavros/battery") is None


def test_fcu_statustext_history_with_chunks():
    client = RadioTelemetryClient(Clock())
    _feed(
        client,
        statustext(FCU, "PreArm: Need Position Estimate"),
        statustext(FCU, "A" * 50, text_id=9, chunk_seq=0),
        statustext(FCU, "B" * 10, text_id=9, chunk_seq=1),
    )
    assert [m["text"] for m in client.statustext_history()] == [
        "PreArm: Need Position Estimate", "A" * 50 + "B" * 10,
    ]


def test_chunked_text_with_a_lost_chunk_is_dropped():
    client = RadioTelemetryClient(Clock())
    _feed(
        client,
        statustext(FCU, "A" * 50, text_id=3, chunk_seq=0),
        statustext(FCU, "C" * 5, text_id=3, chunk_seq=2),
    )
    assert client.statustext_history() == []


def test_statustext_history_is_bounded():
    client = RadioTelemetryClient(Clock())
    _feed(client, *[statustext(FCU, f"t{i}") for i in range(25)])
    history = client.statustext_history()
    assert len(history) == 20 and history[-1]["text"] == "t24"


def test_hover_status_built_from_radio():
    client = RadioTelemetryClient(Clock())
    _feed(
        client,
        jetson_heartbeat(5),
        fcu_heartbeat(armed=True, guided=True, mode=4),
        position(0.1, 0.2, 0.48),
        named("hv_alt", 0.48), named("hv_tgt", 0.5), named("hv_dur", 10.0), named("hv_elap", 3.5),
        statustext(JETSON, "holding 0.5 m for 10 s", severity=6),
    )
    status = client.latest("/flight_test/status")
    assert status["state"] == "hovering"
    assert status["detail"] == "holding 0.5 m for 10 s"
    assert status["current_altitude_m"] == 0.48
    assert status["target_altitude_m"] == 0.5
    assert status["elapsed_hover_s"] == 3.5
    assert status["armed"] is True and status["flight_mode"] == "GUIDED"
    assert status["current_position"] == [0.1, 0.2, 0.48]
    assert status["execution_mode"] == "real"


def test_mission_steps_appear_in_the_messages_once_per_change():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    _feed(client, jetson_heartbeat(0, mission_code=1), statustext(JETSON, "waiting for START", severity=6))
    clock.t += 5
    _feed(client, jetson_heartbeat(0, mission_code=1), statustext(JETSON, "waiting for START", severity=6))  # resend
    _feed(client, jetson_heartbeat(8, mission_code=1), statustext(JETSON, "operator ABORT -- landing", severity=6))
    assert client.statustext_history() == [
        {"severity": 6, "text": "Hover: waiting for START"},
        {"severity": 4, "text": "Hover: operator ABORT -- landing"},
    ]


def test_param_replies_are_not_mission_messages():
    client = RadioTelemetryClient(Clock())
    _feed(client, jetson_heartbeat(0, mission_code=2), statustext(JETSON, "PARAM: X: refused", severity=4))
    assert client.statustext_history() == []


def test_hover_status_absent_without_link_or_mission():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    assert client.latest("/flight_test/status") is None
    _feed(client, jetson_heartbeat(255))  # mission node not running
    assert client.latest("/flight_test/status") is None
    _feed(client, jetson_heartbeat(0), named("hv_tgt", 0.5))
    clock.t += 4.0
    assert client.latest("/flight_test/status") is None


def test_hover_values_expire_individually():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    _feed(client, jetson_heartbeat(5), named("hv_elap", 9.0))
    clock.t += 2.0
    _feed(client, jetson_heartbeat(6), named("hv_tgt", 0.5))
    clock.t += 2.0
    _feed(client, jetson_heartbeat(6))
    status = client.latest("/flight_test/status")
    assert status["elapsed_hover_s"] is None and status["target_altitude_m"] == 0.5


def test_messages_from_other_sources_are_ignored():
    client = RadioTelemetryClient(Clock())
    other = _sender(42)
    client.on_message(_decode(other.heartbeat_encode(2, 3, 129, 4, 4).pack(other)))
    assert client.latest("/mavros/state") is None


def test_client_cannot_publish_anything():
    client = RadioTelemetryClient(Clock())
    for call in (
        lambda: client.publish_command("start"),
        lambda: client.publish_simulation_command("run"),
        lambda: client.publish_mission_select("hover", "hover"),
    ):
        try:
            call()
        except RuntimeError:
            continue
        raise AssertionError("published")


# -- RadioLink hands telemetry over -----------------------------------------------


def test_radio_link_forwards_jetson_and_fcu_messages():
    jetson = FakeJetson()
    serial = FakeSerial(jetson)
    telemetry = RadioTelemetryClient()
    link = RadioLink("COM_TEST", 115200, serial_factory=lambda p, b: serial, telemetry=telemetry)
    link.start()
    try:
        for sender, msg in (jetson_heartbeat(5), fcu_heartbeat(armed=True, mode=4), battery()):
            serial.inbox.put(msg.pack(sender))
        deadline = time.monotonic() + 2
        while telemetry.latest("/mavros/battery") is None and time.monotonic() < deadline:
            time.sleep(0.02)
        status = link.status()
    finally:
        link.stop()
    assert telemetry.is_connected
    assert telemetry.latest("/mavros/state")["mode"] == "GUIDED"
    assert telemetry.latest("/mavros/battery")["voltage"] == 12.345
    # the FCU relay heartbeat must not be mistaken for the Jetson's
    assert status["jetson_mission_state"] == "hovering"


def test_fcu_relay_heartbeat_does_not_count_as_jetson_link():
    jetson = FakeJetson()
    serial = FakeSerial(jetson)
    link = RadioLink("COM_TEST", 115200, serial_factory=lambda p, b: serial, telemetry=RadioTelemetryClient())
    link.start()
    try:
        sender, msg = fcu_heartbeat()
        serial.inbox.put(msg.pack(sender))
        time.sleep(0.3)
        status = link.status()
    finally:
        link.stop()
    assert status["jetson_link_up"] is False


# -- API -------------------------------------------------------------------------


def test_default_settings_use_radio_telemetry():
    assert Settings().telemetry_source == "radio"
    app = create_app(settings=Settings())
    with TestClient(app) as client:
        health = client.get("/health").json()
        telemetry = client.get("/api/telemetry").json()
    assert health["telemetry_source"] == "radio"
    assert health["ros_status"] == "disabled"
    assert health["connected"] is False
    assert telemetry["connected"] is False
    assert telemetry["fcu"]["connected"] is None


def test_invalid_telemetry_source_is_refused():
    try:
        Settings(telemetry_source="wifi")
    except ValueError:
        return
    raise AssertionError("accepted")


def test_api_serves_radio_telemetry():
    telemetry = RadioTelemetryClient()
    _feed(
        telemetry,
        jetson_heartbeat(5),
        fcu_heartbeat(armed=True, guided=True, mode=4),
        battery(),
        position(0.1, 0.2, 0.48),
        attitude(),
        statustext(FCU, "EKF3 IMU0 is using optical flow"),
        named("hv_tgt", 0.5),
    )
    app = create_app(client=telemetry, radio=FakeRadioLink(), settings=Settings())
    with TestClient(app) as client:
        body = client.get("/api/telemetry").json()
        flight = client.get("/api/flight-test/status").json()
    assert body["connected"] is True
    assert body["fcu"] == {"connected": True, "armed": True, "guided": True, "mode": "GUIDED", "system_status": 3}
    assert body["battery"]["voltage"] == 12.345
    assert body["pose"]["position"]["z"] == 0.48
    assert body["attitude"]["w"] == 1.0
    assert body["statustext"][-1]["text"] == "EKF3 IMU0 is using optical flow"
    assert body["heartbeat_age_s"] is not None
    assert flight["state"] == "hovering" and flight["target_altitude_m"] == 0.5



def test_motor_test_status_from_radio():
    client = RadioTelemetryClient(Clock())
    _feed(
        client,
        jetson_heartbeat(11, mission_code=2),
        statustext(JETSON, "motor B (2/4) at 8% for 5 s", severity=6),
        named("hv_dur", 5.0),
    )
    status = client.latest("/flight_test/status")
    assert (status["scenario"], status["state"]) == ("motor_test", "testing")
    assert status["detail"] == "motor B (2/4) at 8% for 5 s"


# -- LiDAR ------------------------------------------------------------------------


def lidar_scan(distances):
    return JETSON, JETSON.obstacle_distance_encode(0, 0, distances, 5, 15, 1200, 5.0, 0.0, 12)


def test_lidar_scan_from_radio():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    assert client.latest("/lidar/sectors") is None
    distances = [65535] * 72
    distances[0], distances[18] = 120, 340
    _feed(client, lidar_scan(distances))
    clock.t += 0.5
    scan = client.latest("/lidar/sectors")
    assert scan["distances_cm"][0] == 120 and scan["distances_cm"][18] == 340
    assert scan["distances_cm"][1] is None
    assert (scan["increment_deg"], scan["max_cm"], scan["age_s"]) == (5.0, 1200, 0.5)
    clock.t += 10
    assert client.latest("/lidar/sectors") is None  # stale


def test_api_lidar():
    telemetry = RadioTelemetryClient()
    distances = [65535] * 72
    distances[36] = 500
    _feed(telemetry, lidar_scan(distances))
    app = create_app(client=telemetry, radio=FakeRadioLink(), settings=Settings())
    with TestClient(app) as client:
        body = client.get("/api/lidar").json()
    assert body["available"] is True and body["distances_cm"][36] == 500 and len(body["distances_cm"]) == 72


def test_api_lidar_unavailable_without_scan():
    app = create_app(client=RadioTelemetryClient(), radio=FakeRadioLink(), settings=Settings())
    with TestClient(app) as client:
        assert client.get("/api/lidar").json()["available"] is False


# -- SLAM map (TUNNEL) ---------------------------------------------------------------

import struct  # noqa: E402


def map_rows(cell_cm, ox, oy, w, h, row0, cells):
    nrows = len(cells) // w
    packed = bytearray((len(cells) + 3) // 4)
    for i, v in enumerate(cells):
        packed[i >> 2] |= v << ((i & 3) * 2)
    payload = struct.pack("<HhhBBBB", cell_cm, ox, oy, w, h, row0, nrows) + bytes(packed)
    return JETSON, JETSON.tunnel_encode(255, 190, 0x8001, len(payload), list(payload) + [0] * (128 - len(payload)))


def slam_pose(x_cm, y_cm, yaw_cdeg):
    payload = struct.pack("<hhh", x_cm, y_cm, yaw_cdeg)
    return JETSON, JETSON.tunnel_encode(255, 190, 0x8002, len(payload), list(payload) + [0] * 122)


def test_map_rows_build_an_occupancy_grid():
    client = RadioTelemetryClient(Clock())
    assert client.latest("/map") is None
    _feed(client, map_rows(25, -100, -50, 4, 3, 1, [2, 1, 1, 0]))
    m = client.latest("/map")
    assert m["info"]["resolution"] == 0.25 and (m["info"]["width"], m["info"]["height"]) == (4, 3)
    assert m["info"]["origin"]["position"] == {"x": -1.0, "y": -0.5}
    assert m["data"] == [-1] * 4 + [100, 0, 0, -1] + [-1] * 4


def test_growing_map_keeps_its_cells():
    """While walking the map grows (new origin/size): the cells already
    received stay, shifted to the same world position, instead of the map
    vanishing until every row is re-sent (seen live 2026-10-09)."""
    client = RadioTelemetryClient(Clock())
    _feed(client, map_rows(25, 0, 0, 2, 2, 0, [2, 1, 1, 2]))      # 2x2 at origin (0,0)
    _feed(client, map_rows(25, -25, 0, 3, 3, 2, [0, 0, 0]))       # grew 1 cell left and 1 row up
    m = client.latest("/map")
    assert (m["info"]["width"], m["info"]["height"]) == (3, 3)
    assert m["data"] == [-1, 100, 0,
                         -1, 0, 100,
                         -1, -1, -1]


def test_new_cell_size_starts_fresh():
    client = RadioTelemetryClient(Clock())
    _feed(client, map_rows(25, 0, 0, 2, 2, 0, [2, 2, 2, 2]))
    _feed(client, map_rows(50, 0, 0, 2, 2, 0, [1, 1]))
    assert client.latest("/map")["data"] == [0, 0, -1, -1]


def test_map_after_long_silence_starts_fresh():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    _feed(client, map_rows(25, 0, 0, 2, 2, 0, [2, 2, 2, 2]))
    clock.t += 60  # Jetson restarted: a new SLAM map
    _feed(client, map_rows(25, -25, 0, 3, 2, 0, [1, 1, 1]))
    assert client.latest("/map")["data"] == [0, 0, 0, -1, -1, -1]


def test_slam_pose_and_api_map():
    telemetry = RadioTelemetryClient()
    _feed(telemetry, map_rows(25, 0, 0, 2, 1, 0, [1, 2]), slam_pose(123, -45, 9000))
    assert telemetry.latest("/slam/pose") == {"x": 1.23, "y": -0.45, "yaw_deg": 90.0}
    app = create_app(client=telemetry, radio=FakeRadioLink(), settings=Settings())
    with TestClient(app) as client:
        body = client.get("/api/map").json()
    assert body["data"] == [0, 100] and body["origin_x"] == 0.0
    assert body["robot"] == {"x": 1.23, "y": -0.45, "yaw_deg": 90.0}


def test_corrupt_map_rows_are_ignored():
    client = RadioTelemetryClient(Clock())
    _feed(client, map_rows(25, 0, 0, 4, 1, 1, [1, 1, 1, 1]))  # row 1 of a 1-row map: out of range
    assert client.latest("/map") is None


# -- 1 m arena grid (TUNNEL ARENA_GRID) ----------------------------------------------

import struct  # noqa: E402

_ARENA = struct.Struct("<BBBhHHhhbbBB")


def arena_parts(gen, theta_cdeg, ou_mm, ov_mm, eu, ev, col0, row0, cols, rows, edges, per_part=444):
    """ARENA_GRID payloads as the Jetson sends them (radio_protocol.py)."""
    chunks = [edges[i : i + per_part] for i in range(0, len(edges), per_part)] or [[]]
    out = []
    for part, chunk in enumerate(chunks):
        body = bytearray((len(chunk) + 3) // 4)
        for i, e in enumerate(chunk):
            body[i >> 2] |= e << ((i & 3) * 2)
        payload = _ARENA.pack(gen, part, len(chunks), theta_cdeg, ou_mm, ov_mm, eu, ev, col0, row0, cols, rows) + body
        out.append((JETSON, JETSON.tunnel_encode(255, 190, 0x8003, len(payload), list(payload) + [0] * (128 - len(payload)))))
    return out


def test_arena_grid_from_one_part():
    client = RadioTelemetryClient(clock=Clock())
    # 2 cols x 1 row: west edges [wall, open, wall], south [wall, unknown], north [wall, wall]
    edges = [2, 1, 2] + [2, 0] + [2, 2]
    _feed(client, *arena_parts(1, 700, 600, 400, -3, 2, 0, 0, 2, 1, edges))
    a = client.latest("/arena/grid")
    assert a["theta_deg"] == 7.0 and a["offset_u"] == 0.6 and a["offset_v"] == 0.4
    assert (a["entry_u"], a["entry_v"], a["col0"], a["row0"], a["cols"], a["rows"]) == (-3, 2, 0, 0, 2, 1)
    assert a["v_edges"] == [[2, 1, 2]]
    assert a["h_edges"] == [[2, 0], [2, 2]]
    assert a["age_s"] == 0.0


def test_arena_grid_waits_for_all_parts_and_drops_a_superseded_one():
    client = RadioTelemetryClient(clock=Clock())
    cols = rows = 15
    edges = [2] * (rows * (cols + 1)) + [1] * ((rows + 1) * cols)
    parts = arena_parts(3, 0, 0, 0, 0, 0, 0, 0, cols, rows, edges)
    assert len(parts) == 2
    _feed(client, parts[1])
    assert client.latest("/arena/grid") is None
    # part 0 of a NEWER grid: the old half is dropped, still nothing complete
    newer = arena_parts(4, 0, 0, 0, 0, 0, 0, 0, cols, rows, [1] * len(edges))
    _feed(client, newer[0])
    assert client.latest("/arena/grid") is None
    _feed(client, newer[1])
    a = client.latest("/arena/grid")
    assert a["v_edges"][0] == [1] * 16 and len(a["h_edges"]) == 16


def test_corrupt_arena_parts_are_ignored():
    client = RadioTelemetryClient(clock=Clock())
    _feed(client, (JETSON, JETSON.tunnel_encode(255, 190, 0x8003, 5, [1, 2, 3, 4, 5] + [0] * 123)))
    bad = _ARENA.pack(1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1)  # part 2 of 1
    _feed(client, (JETSON, JETSON.tunnel_encode(255, 190, 0x8003, len(bad), list(bad) + [0] * (128 - len(bad)))))
    assert client.latest("/arena/grid") is None


def test_api_map_carries_the_arena_grid():
    telemetry = RadioTelemetryClient()
    edges = [2, 2] + [1, 1]
    _feed(telemetry, map_rows(25, 0, 0, 2, 1, 0, [1, 2]), *arena_parts(1, -250, 100, 900, 0, 0, 0, 0, 1, 1, edges))
    app = create_app(client=telemetry, radio=FakeRadioLink(), settings=Settings())
    with TestClient(app) as client:
        arena = client.get("/api/map").json()["arena"]
    assert arena["theta_deg"] == -2.5 and arena["v_edges"] == [[2, 2]] and arena["h_edges"] == [[1], [1]]


# -- Components panel ------------------------------------------------------------------


def by_key(components):
    return {c["key"]: c for c in components}


def test_components_missing_until_heard():
    c = by_key(RadioTelemetryClient(Clock()).components())
    assert list(c) == ["jetson", "fcu", "battery", "position", "orientation", "lidar", "map"]
    assert all(v["status"] == "missing" and v["age_s"] is None for v in c.values())


def test_components_ok_with_details_then_lost():
    clock = Clock()
    client = RadioTelemetryClient(clock)
    _feed(client, jetson_heartbeat(0), fcu_heartbeat(armed=True, mode=4), battery(mv=15800),
          position(), attitude(), lidar_scan([500] * 72), map_rows(25, 0, 0, 4, 3, 0, [1] * 12))
    c = by_key(client.components())
    assert all(v["status"] == "ok" for v in c.values())
    assert c["fcu"]["detail"] == "GUIDED, ARMED"
    assert c["battery"]["detail"] == "15.80 V"
    assert c["map"]["detail"] == "4 x 3 cells @ 0.25 m"
    clock.t += 6.0  # past every limit but the map's
    c = by_key(client.components())
    assert c["lidar"]["status"] == "lost" and c["lidar"]["age_s"] == 6.0 and c["lidar"]["detail"] is None
    assert c["fcu"]["status"] == "lost" and c["map"]["status"] == "ok"


def test_api_components():
    telemetry = RadioTelemetryClient()
    _feed(telemetry, jetson_heartbeat(0), fcu_heartbeat())
    app = create_app(client=telemetry, radio=FakeRadioLink(), settings=Settings())
    with TestClient(app) as client:
        body = client.get("/api/components").json()
    assert body["available"] is True
    c = by_key(body["components"])
    assert c["jetson"]["status"] == "ok" and c["fcu"]["status"] == "ok" and c["lidar"]["status"] == "missing"

"""Unit tests for the radio telemetry encoders (telem_command_codec.py)
and the relay's scheduling rules (radio_telemetry.py). The pymavlink
cross-checks are skipped where pymavlink isn't installed (it isn't on the
Jetson); run them on a dev machine with `pip install pymavlink`."""
import math

import pytest

from nidar_autonomy import radio_telemetry
from nidar_autonomy.radio_telemetry import TelemetryRelay
from nidar_autonomy.telem_command_codec import (
    ARDUCOPTER_MODE_UNKNOWN,
    CRC_EXTRA,
    FCU_RELAY_COMPONENT_ID,
    JETSON_COMPONENT_ID,
    JETSON_SYSTEM_ID,
    MSG_ID_ATTITUDE_QUATERNION,
    MSG_ID_HEARTBEAT,
    MSG_ID_LOCAL_POSITION_NED,
    MSG_ID_NAMED_VALUE_FLOAT,
    MSG_ID_STATUSTEXT,
    MSG_ID_SYS_STATUS,
    MavlinkStreamParser,
    arducopter_mode_number,
    encode_attitude_quaternion,
    encode_fcu_heartbeat,
    encode_local_position,
    encode_named_value_float,
    encode_statustext,
    encode_sys_status,
    statustext_chunks,
)


@pytest.fixture
def mav():
    return pytest.importorskip("pymavlink.dialects.v20.common")


def _parse(mav, raw: bytes):
    msg = mav.MAVLink(None).parse_char(raw)
    assert msg is not None
    return msg


class TestEncodersAgainstPymavlink:
    def test_crc_extras_match_pymavlink(self, mav):
        for msgid, crc in CRC_EXTRA.items():
            assert mav.mavlink_map[msgid].crc_extra == crc, msgid

    def test_fcu_heartbeat(self, mav):
        msg = _parse(mav, encode_fcu_heartbeat(True, True, "GUIDED", 4, seq=3))
        assert (msg.get_srcSystem(), msg.get_srcComponent()) == (JETSON_SYSTEM_ID, FCU_RELAY_COMPONENT_ID)
        assert msg.custom_mode == 4
        assert msg.base_mode & 128 and msg.base_mode & 8
        assert (msg.type, msg.autopilot, msg.system_status) == (2, 3, 4)

    def test_fcu_heartbeat_disarmed_unknown_mode(self, mav):
        msg = _parse(mav, encode_fcu_heartbeat(False, False, "CMODE(99)", 3, seq=0))
        assert msg.custom_mode == ARDUCOPTER_MODE_UNKNOWN
        assert not msg.base_mode & 128

    def test_mode_table_matches_pymavlink(self):
        mavutil = pytest.importorskip("pymavlink.mavutil")
        for number, name in mavutil.mode_mapping_acm.items():
            if name in ("POSITION", "OF_LOITER", "RATE_ACRO"):
                continue  # not ArduCopter 4.x modes
            assert arducopter_mode_number(name) == number, name

    def test_sys_status_battery(self, mav):
        msg = _parse(mav, encode_sys_status(12.345, -3.21, 0.87, seq=1))
        assert (msg.voltage_battery, msg.current_battery, msg.battery_remaining) == (12345, -321, 87)

    def test_sys_status_unknowns(self, mav):
        msg = _parse(mav, encode_sys_status(math.nan, None, math.nan, seq=1))
        assert (msg.voltage_battery, msg.current_battery, msg.battery_remaining) == (65535, -1, -1)

    def test_sys_status_zero_volts_is_sent_as_zero_not_unknown(self, mav):
        msg = _parse(mav, encode_sys_status(0.0, 0.0, 0.0, seq=1))
        assert (msg.voltage_battery, msg.current_battery, msg.battery_remaining) == (0, 0, 0)

    def test_local_position(self, mav):
        msg = _parse(mav, encode_local_position(1500, (1.0, -2.5, 0.5), (0.1, 0.2, -0.3), seq=2))
        assert msg.time_boot_ms == 1500
        assert (msg.x, msg.y, msg.z) == pytest.approx((1.0, -2.5, 0.5))
        assert (msg.vx, msg.vy, msg.vz) == pytest.approx((0.1, 0.2, -0.3))

    def test_local_position_without_velocity_sends_nan(self, mav):
        msg = _parse(mav, encode_local_position(0, (0.0, 0.0, 0.0), None, seq=2))
        assert math.isnan(msg.vx) and math.isnan(msg.vz)
        assert msg.x == 0.0

    def test_attitude_quaternion(self, mav):
        msg = _parse(mav, encode_attitude_quaternion(10, 0.7071, 0.0, 0.0, 0.7071, seq=4))
        assert (msg.q1, msg.q2, msg.q3, msg.q4) == pytest.approx((0.7071, 0.0, 0.0, 0.7071))

    def test_named_value_float(self, mav):
        msg = _parse(mav, encode_named_value_float(5, "hv_alt", 0.42, seq=5))
        assert msg.name == "hv_alt"
        assert msg.value == pytest.approx(0.42)
        assert msg.get_srcComponent() == JETSON_COMPONENT_ID

    def test_statustext_single(self, mav):
        msg = _parse(mav, encode_statustext(4, b"PreArm: Need Position Estimate", seq=6))
        assert (msg.severity, msg.text, msg.id, msg.chunk_seq) == (4, "PreArm: Need Position Estimate", 0, 0)

    def test_statustext_chunk(self, mav):
        msg = _parse(mav, encode_statustext(6, b"x" * 50, seq=6, text_id=7, chunk_seq=1))
        assert (msg.text, msg.id, msg.chunk_seq) == ("x" * 50, 7, 1)


class TestStatustextChunks:
    def test_short_text_is_one_chunk(self):
        assert statustext_chunks("hello") == [b"hello"]

    def test_exactly_50_is_one_chunk(self):
        assert statustext_chunks("a" * 50) == [b"a" * 50]

    def test_long_text_is_split(self):
        assert statustext_chunks("a" * 60) == [b"a" * 50, b"a" * 10]

    def test_exact_multiple_gets_empty_terminator(self):
        assert statustext_chunks("a" * 100) == [b"a" * 50, b"a" * 50, b""]

    def test_non_ascii_is_replaced_not_crashing(self):
        assert statustext_chunks("alt 0.5 m → ok") == [b"alt 0.5 m ? ok"]


# -- relay ---------------------------------------------------------------------


class Seq:
    def __init__(self):
        self.n = 0

    def __call__(self):
        self.n = (self.n + 1) & 0xFF
        return self.n


def _frames(raw_frames):
    return MavlinkStreamParser().feed(b"".join(raw_frames))


def _ids(raw_frames):
    return [(f.msgid, f.compid) for f in _frames(raw_frames)]


def _relay():
    return TelemetryRelay(Seq(), start_time=100.0)


def _fill(relay, now):
    relay.update_fcu_state(now, True, False, False, "STABILIZE", 3)
    relay.update_battery(now, 12.0, -1.0, 0.9)
    relay.update_position(now, 0.0, 0.0, 0.1)
    relay.update_velocity(now, 0.0, 0.0, 0.0)
    relay.update_orientation(now, 1.0, 0.0, 0.0, 0.0)


def test_nothing_is_sent_before_any_input():
    assert _relay().tick(100.0) == []


def test_first_tick_sends_everything_fresh():
    relay = _relay()
    _fill(relay, 100.0)
    ids = _ids(relay.tick(100.0))
    assert (MSG_ID_LOCAL_POSITION_NED, FCU_RELAY_COMPONENT_ID) in ids
    assert (MSG_ID_ATTITUDE_QUATERNION, FCU_RELAY_COMPONENT_ID) in ids
    assert (MSG_ID_HEARTBEAT, FCU_RELAY_COMPONENT_ID) in ids
    assert (MSG_ID_SYS_STATUS, FCU_RELAY_COMPONENT_ID) in ids


def test_slow_messages_go_once_a_second_fast_ones_every_tick():
    relay = _relay()
    _fill(relay, 100.0)
    relay.tick(100.0)
    _fill(relay, 100.5)
    ids = _ids(relay.tick(100.5))
    assert (MSG_ID_LOCAL_POSITION_NED, FCU_RELAY_COMPONENT_ID) in ids
    assert (MSG_ID_HEARTBEAT, FCU_RELAY_COMPONENT_ID) not in ids
    assert (MSG_ID_SYS_STATUS, FCU_RELAY_COMPONENT_ID) not in ids
    _fill(relay, 101.0)
    assert (MSG_ID_HEARTBEAT, FCU_RELAY_COMPONENT_ID) in _ids(relay.tick(101.0))


def test_stale_inputs_are_not_sent():
    relay = _relay()
    _fill(relay, 100.0)
    ids = _ids(relay.tick(110.0))
    assert ids == []


def test_fcu_heartbeat_only_while_mavros_connected():
    relay = _relay()
    relay.update_fcu_state(100.0, False, False, False, "", 0)
    assert _ids(relay.tick(100.0)) == []


def test_position_without_fresh_velocity_still_sent():
    relay = _relay()
    relay.update_position(100.0, 1.0, 2.0, 3.0)
    frames = _frames(relay.tick(100.0))
    assert [f.msgid for f in frames] == [MSG_ID_LOCAL_POSITION_NED]


def test_hover_values_and_detail_from_mission_status():
    relay = _relay()
    relay.update_mission_status(100.0, {
        "state": "hovering", "detail": "holding 0.5 m", "current_altitude_m": 0.48,
        "target_altitude_m": 0.5, "duration_s": 10.0, "elapsed_hover_s": None,
    })
    ids = _ids(relay.tick(100.0))
    assert ids.count((MSG_ID_NAMED_VALUE_FLOAT, JETSON_COMPONENT_ID)) == 3  # elapsed is None
    assert (MSG_ID_STATUSTEXT, JETSON_COMPONENT_ID) in ids


def test_mission_detail_is_resent_periodically_and_on_change():
    relay = _relay()
    status = {"state": "idle", "detail": "waiting for START"}

    def texts(now):
        relay.update_mission_status(now, status)
        return _ids(relay.tick(now)).count((MSG_ID_STATUSTEXT, JETSON_COMPONENT_ID))

    assert texts(100.0) == 1
    assert texts(101.0) == 0
    assert texts(100.0 + radio_telemetry.MISSION_DETAIL_RESEND_S) == 1
    status = {"state": "preflight", "detail": "checking"}
    assert texts(106.0) == 1


def test_fcu_statustext_is_rate_limited_and_in_order():
    relay = _relay()
    for i in range(5):
        relay.add_fcu_statustext(4, f"msg {i}")
    sent = []
    for k in range(3):
        frames = [f for f in _frames(relay.tick(100.0 + k)) if f.msgid == MSG_ID_STATUSTEXT]
        assert len(frames) <= radio_telemetry.STATUSTEXT_PER_TICK
        assert all(f.compid == FCU_RELAY_COMPONENT_ID for f in frames)
        sent += [f.payload[1:].split(b"\x00")[0].decode() for f in frames]
    assert sent == [f"msg {i}" for i in range(5)]


def test_long_text_chunks_share_an_id_and_continue_next_tick():
    mav = pytest.importorskip("pymavlink.dialects.v20.common")
    relay = _relay()
    relay.add_fcu_statustext(3, "E" * 120)
    raw = relay.tick(100.0) + relay.tick(101.0)
    parser = mav.MAVLink(None)
    msgs = [m for m in (parser.parse_buffer(b"".join(raw)) or []) if m.get_type() == "STATUSTEXT"]
    assert [m.chunk_seq for m in msgs] == [0, 1, 2]
    assert len({m.id for m in msgs}) == 1 and msgs[0].id != 0
    assert "".join(m.text for m in msgs) == "E" * 120


def test_statustext_queue_is_bounded():
    relay = _relay()
    for i in range(100):
        relay.add_fcu_statustext(6, f"t{i}")
    total = 0
    for k in range(100):
        total += sum(1 for f in _frames(relay.tick(100.0 + k)) if f.msgid == MSG_ID_STATUSTEXT)
    assert total == radio_telemetry.STATUSTEXT_QUEUE


def test_load_stays_small_at_default_rate():
    """Every input fresh, 2 Hz for 10 s: well under 300 B/s, so START/ABORT
    ACKs are never queued behind telemetry on a slow radio."""
    relay = _relay()
    total = 0
    for k in range(20):
        now = 100.0 + k * 0.5
        _fill(relay, now)
        relay.update_mission_status(now, {"state": "hovering", "detail": "holding", "current_altitude_m": 0.5,
                                          "target_altitude_m": 0.5, "duration_s": 10.0, "elapsed_hover_s": 1.0})
        total += sum(len(f) for f in relay.tick(now))
    assert total / 10.0 < 300


# -- LiDAR (OBSTACLE_DISTANCE) ------------------------------------------------------

from nidar_autonomy.telem_command_codec import MSG_ID_OBSTACLE_DISTANCE, encode_obstacle_distance  # noqa: E402

SECTORS = [100 + i for i in range(72)]


def test_obstacle_distance_parses_in_pymavlink(mav):
    raw = encode_obstacle_distance(123456, SECTORS, 15, 1200, 5.0, seq=4)
    msg = _parse(mav, raw)
    assert list(msg.distances) == SECTORS
    assert (msg.min_distance, msg.max_distance, msg.increment, msg.increment_f) == (15, 1200, 5, 5.0)
    assert msg.frame == 12 and msg.sensor_type == 0
    assert (msg.get_srcSystem(), msg.get_srcComponent()) == (JETSON_SYSTEM_ID, JETSON_COMPONENT_ID)
    assert len(raw) < 190


def test_obstacle_distance_unknown_sectors_survive(mav):
    sectors = [65535] * 72
    sectors[0] = 250
    msg = _parse(mav, encode_obstacle_distance(0, sectors, 15, 1200, 5.0, seq=0))
    assert msg.distances[0] == 250 and msg.distances[1] == 65535


def test_relay_sends_scan_at_lidar_rate_only_while_fresh():
    relay = TelemetryRelay(Seq(), start_time=100.0, lidar_rate_hz=1.0)

    def scans(now):
        return [f for f in _frames(relay.tick(now)) if f.msgid == MSG_ID_OBSTACLE_DISTANCE]

    relay.update_scan(100.0, SECTORS, 15, 1200)
    assert len(scans(100.0)) == 1
    relay.update_scan(100.4, SECTORS, 15, 1200)
    assert scans(100.5) == []  # rate limit: 1 per second
    relay.update_scan(100.9, SECTORS, 15, 1200)
    assert len(scans(101.0)) == 1
    assert scans(103.0) == []  # scan older than 1 s: not sent


def test_relay_lidar_off_when_rate_zero():
    relay = TelemetryRelay(Seq(), start_time=100.0, lidar_rate_hz=0)
    relay.update_scan(100.0, SECTORS, 15, 1200)
    assert all(f.msgid != MSG_ID_OBSTACLE_DISTANCE for f in _frames(relay.tick(100.0)))


def test_load_with_lidar_at_default_rate():
    """Everything fresh incl. LiDAR at 1 Hz, telemetry at 2 Hz: under 500 B/s."""
    relay = _relay()
    total = 0
    for k in range(20):
        now = 100.0 + k * 0.5
        _fill(relay, now)
        relay.update_scan(now, SECTORS, 15, 1200)
        total += sum(len(f) for f in relay.tick(now))
    assert total / 10.0 < 500


# -- rates under tick jitter (Jetson measured 0.77 Hz for "1 Hz", 2026-10-09) -----

import random  # noqa: E402

from nidar_autonomy.radio_telemetry import Schedule  # noqa: E402
from nidar_autonomy.telem_command_codec import FCU_RELAY_COMPONENT_ID as _FCU  # noqa: E402


def _jittered_run(seconds=120.0, tick=0.5, jitter=0.004, lidar_rate=1.0, seed=7):
    rng = random.Random(seed)
    relay = TelemetryRelay(Seq(), start_time=0.0, lidar_rate_hz=lidar_rate)
    counts = {"scan": 0, "fcu_heartbeat": 0}
    t = 0.0
    while t < seconds:
        now = t + rng.uniform(-jitter, jitter)
        _fill(relay, now)
        relay.update_scan(now, SECTORS, 15, 1200)
        for f in _frames(relay.tick(now)):
            if f.msgid == MSG_ID_OBSTACLE_DISTANCE:
                counts["scan"] += 1
            elif f.msgid == MSG_ID_HEARTBEAT and f.compid == _FCU:
                counts["fcu_heartbeat"] += 1
        t += tick
    return counts


def test_one_hz_stays_one_hz_with_jittered_ticks():
    counts = _jittered_run()
    assert 119 <= counts["scan"] <= 121
    assert 119 <= counts["fcu_heartbeat"] <= 121


def test_half_hz_lidar_with_jitter():
    assert 59 <= _jittered_run(lidar_rate=0.5)["scan"] <= 61


def test_schedule_restarts_after_a_pause():
    s = Schedule(1.0)
    assert s.due(0.0) and not s.due(0.5) and s.due(1.0)
    assert s.due(10.0)  # long gap: no burst of catch-up sends
    assert not s.due(10.5) and s.due(11.0)


def test_schedule_accepts_slightly_early_ticks():
    s = Schedule(1.0)
    assert s.due(0.0)
    assert s.due(0.999)  # 1 ms early still counts
    assert not s.due(1.5)
    assert s.due(2.0)


# -- rangefinder (DISTANCE_SENSOR -> NAMED_VALUE_FLOAT "rngfnd") ---------------------

def test_decode_distance_sensor_reads_metres_and_orientation():
    import struct
    from nidar_autonomy.telem_command_codec import decode_distance_sensor
    full = struct.pack("<IHHHBBBB", 1234, 5, 800, 13, 0, 0, 25, 0)
    assert decode_distance_sensor(full) == (0.13, 25)
    # MAVLink 2 trims trailing zero bytes: still decodes
    assert decode_distance_sensor(full.rstrip(b"\x00")) == (0.13, 25)
    assert decode_distance_sensor(b"\x00" * 6) is None


def test_rangefinder_is_sent_every_tick_while_fresh():
    from pymavlink.dialects.v20 import common as mavlink
    relay = TelemetryRelay(Seq(), start_time=100.0)

    def rangefinder_values(frames):
        out = []
        for frame in frames:
            m = mavlink.MAVLink(None).parse_char(frame)
            if m is not None and m.get_type() == "NAMED_VALUE_FLOAT" and m.name == "rngfnd":
                out.append(round(m.value, 2))
        return out

    relay.update_rangefinder(110.0, 0.13)
    assert rangefinder_values(relay.tick(110.1)) == [0.13]
    assert rangefinder_values(relay.tick(110.3)) == [0.13]  # every tick, not 1 Hz
    assert rangefinder_values(relay.tick(112.0)) == []  # stale: not sent

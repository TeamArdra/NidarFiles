"""Altitude panel data: every radio LOCAL_POSITION_NED height kept with its
time, and the height when the FCU armed (= take-off height)."""
from __future__ import annotations

from fastapi.testclient import TestClient
from pymavlink.dialects.v20 import common as mavlink

from app.config import Settings
from app.main import create_app
from app.radio_telemetry import ALTITUDE_HISTORY_S, RadioTelemetryClient

from .fakes import FakeRadioLink
from .test_radio_telemetry import FCU, Clock, _feed, fcu_heartbeat


def position(z, vz=0.0):
    # the Jetson sends the ROS ENU pose unchanged in LOCAL_POSITION_NED
    return FCU, FCU.local_position_ned_encode(0, 1.0, 2.0, z, 0.0, 0.0, vz)


def test_every_height_is_kept_with_its_time():
    clock = Clock()
    client = RadioTelemetryClient(clock=clock)
    for i, z in enumerate((0.10, 0.20, 0.35)):
        clock.t = 1000.0 + 0.2 * i
        _feed(client, position(z, 0.5))
    out = client.altitude()
    assert [s[1] for s in out["samples"]] == [0.1, 0.2, 0.35]
    assert out["samples"][0][0] == 1000.0 and out["samples"][0][2] == 0.5
    # only newer samples when asked "since"
    assert [s[1] for s in client.altitude(since=1000.2)["samples"]] == [0.35]


def test_ground_height_is_taken_when_the_fcu_arms():
    clock = Clock()
    client = RadioTelemetryClient(clock=clock)
    _feed(client, position(1.89), fcu_heartbeat(armed=False))  # baro offset on the ground
    assert client.altitude()["ground_z"] is None
    _feed(client, fcu_heartbeat(armed=True))
    assert client.altitude()["ground_z"] == 1.89
    _feed(client, position(2.40), fcu_heartbeat(armed=True))  # still armed: unchanged
    assert client.altitude()["ground_z"] == 1.89


def test_old_samples_are_dropped():
    clock = Clock()
    client = RadioTelemetryClient(clock=clock)
    _feed(client, position(0.5))
    clock.t += ALTITUDE_HISTORY_S + 1
    _feed(client, position(0.6))
    assert [s[1] for s in client.altitude()["samples"]] == [0.6]


def test_api_altitude():
    clock = Clock()
    client = RadioTelemetryClient(clock=clock)
    _feed(client, position(0.42, -0.1))
    app = create_app(client=client, settings=Settings(radio_enabled=False), radio=FakeRadioLink())
    with TestClient(app) as http:
        body = http.get("/api/altitude").json()
    assert body["available"] is True
    assert body["samples"] == [[1000.0, 0.42, -0.1]]

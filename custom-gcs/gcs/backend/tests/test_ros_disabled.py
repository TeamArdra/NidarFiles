"""ROS-optional local-development mode -- GCS_ROS_ENABLED=false (see
app/config.py's Settings.ros_enabled and app/ros_client.py's
DisabledRosBridgeClient). Covers the scenario this mode exists for: a
developer with no Jetson, no ROS, no rosbridge at all (e.g. a Windows
laptop) needs the FastAPI backend to start, mission/scenario metadata to
be inspectable, and ROS-dependent reads/writes to degrade honestly
instead of crashing the app or faking success.

Uses `create_app(settings=...)` with `client=None` throughout (not
`client=<fake>`) specifically so create_app()'s own client-selection
logic (real RosBridgeClient vs. DisabledRosBridgeClient, based on
settings.ros_enabled) is what's under test, and `with TestClient(app) as
client:` so the real lifespan (startup/shutdown) runs -- this is the
exact code path that used to crash at startup (roslibpy.RosTimeoutError
propagating out of an unconditional ros_client.connect()) before this
mode existed.
"""

from __future__ import annotations

from fastapi.testclient import TestClient

from app.config import Settings
from app.main import create_app

# Nothing should ever be listening here -- used by the "ROS enabled but
# rosbridge unreachable" tests below, deliberately distinct from the
# "ROS disabled" tests (see module docstring: these are two different
# unavailable-ROS scenarios and must both fail safe, not crash).
_UNREACHABLE_PORT = 19199


def _disabled_settings() -> Settings:
    return Settings(ros_enabled=False, telemetry_source="rosbridge")


class TestStartupAndShutdown:
    def test_backend_starts_and_stops_cleanly_with_ros_disabled(self):
        """The bug report this task fixes: FastAPI startup used to fail
        unconditionally (roslibpy.RosTimeoutError from ros_client.connect())
        whenever rosbridge wasn't reachable. With GCS_ROS_ENABLED=false, the
        app must never even attempt a connection, so startup/shutdown must
        complete without raising."""
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/health")
            assert resp.status_code == 200

    def test_ros_enabled_but_rosbridge_unreachable_does_not_crash_startup(self):
        """Separate scenario from ROS-disabled: ROS IS enabled (the
        Jetson-deployment default), but nothing is listening at the
        configured host/port -- e.g. the Jetson is off, rosbridge_server
        hasn't started yet, or (this task's original bug report) a
        developer left GCS_ROS_ENABLED unset on a laptop with no rosbridge
        at all. Startup must log/continue rather than crash the whole
        FastAPI process, exactly as it does for the ROS-disabled case
        above -- just for a different underlying reason."""
        settings = Settings(
            rosbridge_host="127.0.0.1",
            rosbridge_port=_UNREACHABLE_PORT,
            connect_timeout_s=0.3,
            ros_enabled=True,
            telemetry_source="rosbridge",
        )
        app = create_app(settings=settings)
        with TestClient(app) as client:
            resp = client.get("/health")
            assert resp.status_code == 200
            body = resp.json()
            assert body["connected"] is False
            assert body["ros_status"] == "unavailable"


class TestHealthReportsRosStatus:
    def test_disabled_mode_reports_disabled_not_connected_and_not_unavailable(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            body = client.get("/health").json()
        assert body["connected"] is False
        assert body["ros_status"] == "disabled"

    def test_health_still_reports_the_configured_rosbridge_target_when_disabled(self):
        """Even in disabled mode, /health should still report what host/port
        *would* be used if ROS were enabled -- Footer.tsx on the frontend
        reads this unconditionally, and it must not go missing/null just
        because ROS itself is off."""
        settings = Settings(rosbridge_host="10.0.0.5", rosbridge_port=9090, ros_enabled=False, telemetry_source="rosbridge")
        app = create_app(settings=settings)
        with TestClient(app) as client:
            body = client.get("/health").json()
        assert body["rosbridge_host"] == "10.0.0.5"
        assert body["rosbridge_port"] == 9090


class TestMissionRegistryWorksWithoutRos:
    """app/missions.py's registry never touches ros_client at all -- these
    routes must behave identically whether ROS is disabled or not."""

    def test_list_missions(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/api/missions")
        assert resp.status_code == 200
        assert [m["id"] for m in resp.json()] == ["hover", "motor_test"]

    def test_mission_detail(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/api/missions/hover")
        assert resp.status_code == 200

    def test_unknown_mission_still_404s(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/api/missions/does_not_exist")
        assert resp.status_code == 404


class TestRosDependentReadsDegradeInsteadOfCrashing:
    def test_telemetry_returns_all_defaults_not_an_error(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/api/telemetry")
        assert resp.status_code == 200
        body = resp.json()
        assert body["connected"] is False
        assert body["mission_state"] is None
        assert body["battery"] == {"voltage": None, "current": None, "percentage": None}

    def test_survivors_returns_empty_list(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/api/survivors")
        assert resp.status_code == 200
        assert resp.json() == []

    def test_map_returns_empty_snapshot(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/api/map")
        assert resp.status_code == 200
        assert resp.json() == {"resolution": None, "width": None, "height": None, "data": None, "origin_x": None, "origin_y": None, "robot": None, "age_s": None, "arena": None}

    def test_flight_test_status_returns_defaults(self):
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.get("/api/flight-test/status")
        assert resp.status_code == 200
        assert resp.json()["state"] is None


class TestRosDependentWritesFailSafeInsteadOfFakingSuccess:
    """The core safety requirement: nothing may come back looking like it
    was sent when the transport it needs is disabled."""

    def test_radio_disabled_start_and_abort_503_never_a_fake_success(self):
        """START/ABORT go over the command radio, not ROS. With the radio
        disabled (GCS_RADIO_ENABLED=false) they must fail loudly."""
        app = create_app(settings=Settings(ros_enabled=False, radio_enabled=False, telemetry_source="rosbridge"))
        with TestClient(app) as client:
            start = client.post("/api/mission/start", json={"mission": "hover"})
            abort = client.post("/api/command/abort")
        for resp in (start, abort):
            assert resp.status_code == 503
            assert "radio is disabled" in resp.json()["detail"]

    def test_commands_do_not_depend_on_ros(self):
        """ROS disabled but radio up: START still goes out over the radio
        (telemetry being unavailable doesn't block the command path)."""
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.post("/api/mission/start", json={"mission": "hover"})
        assert resp.status_code == 200
        assert resp.json()["status"] == "accepted"

    def test_simulation_run_503s_with_ros_disabled(self):
        """The simulation control surface also goes through ros_client
        (publish_simulation_command) despite being otherwise isolated from
        the real mission -- see app/ros_client.py's SIMULATION_COMMAND_TOPIC
        docstring -- so it must fail the same way, not silently no-op."""
        app = create_app(settings=_disabled_settings())
        with TestClient(app) as client:
            resp = client.post("/api/simulation/run")
        assert resp.status_code == 503
        assert "ROS is disabled" in resp.json()["detail"]

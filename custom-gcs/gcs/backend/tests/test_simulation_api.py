"""Tests for the GCS "RUN SIMULATION" API surface -- POST
/api/simulation/{run,reset} and GET /api/simulation/{status,map,coverage,
path}. The central property under test throughout: this surface is
COMPLETELY separate from the real mission command/telemetry surface --
see app/main.py's module docstring and ros_client.py's
publish_simulation_command()."""
from fastapi.testclient import TestClient

from app.main import create_app

from .fakes import FakeRadioLink, FakeRosBridgeClient


def make_client() -> tuple[TestClient, FakeRosBridgeClient]:
    fake = FakeRosBridgeClient()
    app = create_app(client=fake)
    return TestClient(app), fake


class TestSimulationDoesNotTouchTheRealCommandPath:
    def test_run_never_calls_the_real_publish_command(self):
        client, fake = make_client()
        resp = client.post("/api/simulation/run")
        assert resp.status_code == 200
        assert fake.published_commands == []
        assert fake.published_simulation_commands == ["run"]

    def test_reset_never_calls_the_real_publish_command(self):
        client, fake = make_client()
        resp = client.post("/api/simulation/reset")
        assert resp.status_code == 200
        assert fake.published_commands == []
        assert fake.published_simulation_commands == ["reset"]

    def test_real_start_and_abort_never_touch_simulation_command(self):
        fake = FakeRosBridgeClient()
        radio = FakeRadioLink()
        client = TestClient(create_app(client=fake, radio=radio))
        client.post("/api/mission/start", json={"mission": "hover"})
        client.post("/api/simulation/run")
        client.post("/api/command/abort")
        client.post("/api/simulation/reset")
        assert radio.sent == [("start", 1), ("abort", 0)]
        assert fake.published_commands == []
        assert fake.published_simulation_commands == ["run", "reset"]

    def test_run_response_shape(self):
        client, _ = make_client()
        body = client.post("/api/simulation/run").json()
        assert body == {"status": "sent", "command": "run"}

    def test_reset_response_shape(self):
        client, _ = make_client()
        body = client.post("/api/simulation/reset").json()
        assert body == {"status": "sent", "command": "reset"}

    def test_run_surfaces_a_structured_error_when_not_connected(self):
        client, fake = make_client()
        fake.fail_publish_simulation_with(RuntimeError("not connected to rosbridge"))
        resp = client.post("/api/simulation/run")
        assert resp.status_code == 503

    def test_no_real_command_vocabulary_accepted_on_the_simulation_path(self):
        """/api/simulation/start and /api/simulation/abort must not
        exist -- "start"/"abort" stay exclusively the real command
        surface's vocabulary."""
        client, _ = make_client()
        assert client.post("/api/simulation/start").status_code == 404
        assert client.post("/api/simulation/abort").status_code == 404

    def test_no_simulation_vocabulary_accepted_on_the_real_command_path(self):
        """/api/command/run and /api/command/reset must not exist --
        "run"/"reset" stay exclusively the simulation surface's
        vocabulary."""
        client, _ = make_client()
        assert client.post("/api/command/run").status_code == 404
        assert client.post("/api/command/reset").status_code == 404


class TestSimulationStatus:
    def test_defaults_before_any_simulation_data_received(self):
        client, _ = make_client()
        body = client.get("/api/simulation/status").json()
        assert body["source"] == "simulation"
        assert body["status"] == "idle"
        assert body["mission_state"] == "idle"
        assert body["step"] == 0
        assert body["pose"] is None

    def test_reflects_real_cached_simulation_telemetry(self):
        client, fake = make_client()
        fake.set_latest("/simulation/status", {"data": "running"})
        fake.set_latest("/simulation/mission/state", {"data": "searching"})
        fake.set_latest(
            "/simulation/telemetry/state",
            {
                "source": "simulation",
                "simulation_step": 42,
                "position": {"x": 3.5, "y": 2.5, "z": 1.0},
                "mission": {"elapsed_sec": 21.0},
                "autonomy": {"state": "SEARCHING_FRONTIER", "objective": "Explore unexplored region",
                             "target": [5.0, 3.0], "next_action": "Navigate to frontier"},
                "mapping": {"available": True, "explored_pct": 42.3},
                "navigation": {"target": [5.0, 3.0], "frontier_count": 2, "candidate_count": 3,
                               "blacklisted_count": 0, "geofence_breached": False},
                "coverage_search_pct": 88.0,
                "error": None,
            },
        )

        body = client.get("/api/simulation/status").json()

        assert body["status"] == "running"
        assert body["mission_state"] == "searching"
        assert body["step"] == 42
        assert body["pose"] == {"x": 3.5, "y": 2.5, "z": 1.0}
        assert body["autonomy"]["state"] == "SEARCHING_FRONTIER"
        assert body["map_known_pct"] == 42.3
        assert body["coverage_search_pct"] == 88.0

    def test_status_response_never_looks_like_real_telemetry_shape(self):
        """Structural guarantee: SimulationStatusResponse always carries
        source == "simulation", and the real /api/telemetry response has
        no such field at all -- the two payloads can never be confused
        even by a consumer that doesn't check types."""
        client, _ = make_client()
        sim_body = client.get("/api/simulation/status").json()
        real_body = client.get("/api/telemetry").json()
        assert sim_body["source"] == "simulation"
        assert "source" not in real_body


class TestSimulationMapCoveragePath:
    def test_map_before_any_data(self):
        client, _ = make_client()
        body = client.get("/api/simulation/map").json()
        assert body == {"resolution": None, "width": None, "height": None, "data": None, "origin_x": None, "origin_y": None, "robot": None, "age_s": None, "arena": None}

    def test_map_reflects_simulation_topic_not_real_map_topic(self):
        client, fake = make_client()
        fake.set_latest(
            "/map",  # the REAL map topic -- must be ignored by the simulation route
            {"info": {"resolution": 1.0, "width": 99, "height": 99}, "data": [0] * (99 * 99)},
        )
        fake.set_latest(
            "/simulation/map",
            {"info": {"resolution": 1.0, "width": 5, "height": 5}, "data": [-1] * 25},
        )
        body = client.get("/api/simulation/map").json()
        assert body["width"] == 5  # from /simulation/map, not the real /map's 99

    def test_coverage_reflects_simulation_topic(self):
        client, fake = make_client()
        fake.set_latest(
            "/simulation/coverage_grid",
            {"info": {"resolution": 1.0, "width": 3, "height": 3}, "data": [100] * 9},
        )
        body = client.get("/api/simulation/coverage").json()
        assert body["width"] == 3
        assert body["data"] == [100] * 9

    def test_path_reflects_simulation_topic(self):
        client, fake = make_client()
        fake.set_latest(
            "/simulation/planned_path",
            {"poses": [{"pose": {"position": {"x": 1.0, "y": 2.0}}}]},
        )
        body = client.get("/api/simulation/path").json()
        assert body["points"] == [{"x": 1.0, "y": 2.0}]

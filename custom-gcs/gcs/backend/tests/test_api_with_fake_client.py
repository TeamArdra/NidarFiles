from fastapi.testclient import TestClient

from app.main import create_app

from .fakes import FakeRadioLink, FakeRosBridgeClient


def make_client() -> tuple[TestClient, FakeRosBridgeClient]:
    fake = FakeRosBridgeClient()
    app = create_app(client=fake)
    return TestClient(app), fake


def make_radio_client() -> tuple[TestClient, FakeRosBridgeClient, FakeRadioLink]:
    fake = FakeRosBridgeClient()
    radio = FakeRadioLink()
    return TestClient(create_app(client=fake, radio=radio)), fake, radio


def test_health_reports_connection_state():
    client, _ = make_client()
    resp = client.get("/health")
    assert resp.status_code == 200
    assert resp.json()["connected"] is True


def test_telemetry_is_all_none_before_any_data_arrives():
    client, _ = make_client()
    body = client.get("/api/telemetry").json()
    assert body["mission_state"] is None
    assert body["battery"] == {"voltage": None, "current": None, "percentage": None}
    assert body["pose"]["position"] is None
    assert body["fcu"] == {
        "connected": None,
        "armed": None,
        "guided": None,
        "mode": None,
        "system_status": None,
    }
    assert body["velocity"] is None
    assert body["attitude"] is None
    assert body["gps"] is None
    assert body["statustext"] == []
    assert body["heartbeat_age_s"] is None


def test_telemetry_reflects_latest_cached_values():
    client, fake = make_client()
    fake.set_latest("/mission/state", {"data": "searching"})
    fake.set_latest("/mavros/battery", {"voltage": 15.2, "current": 2.1, "percentage": 0.62})
    fake.set_latest(
        "/mavros/local_position/pose",
        {"pose": {"position": {"x": 3.0, "y": 4.0, "z": 0.0}}},
    )
    fake.set_latest(
        "/mavros/state",
        {"connected": True, "armed": True, "guided": False, "mode": "STABILIZE", "system_status": 4},
    )
    fake.set_latest(
        "/mavros/local_position/velocity_local",
        {"twist": {"linear": {"x": 0.1, "y": 0.0, "z": 0.0}}},
    )
    fake.set_latest(
        "/mavros/global_position/global",
        {"status": {"status": 0}, "satellites_visible": 8, "latitude": 1.0, "longitude": 2.0, "altitude": 3.0},
    )
    fake.set_latest(
        "/mavros/imu/data",
        {"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
    )
    fake.set_statustext_history([{"severity": 6, "text": "Arming Checks Disabled"}])
    fake.set_age_s("/gcs/heartbeat", 0.4)

    body = client.get("/api/telemetry").json()

    assert body["mission_state"] == "searching"
    assert body["battery"] == {"voltage": 15.2, "current": 2.1, "percentage": 0.62}
    assert body["pose"]["position"] == {"x": 3.0, "y": 4.0, "z": 0.0}
    assert body["fcu"] == {
        "connected": True,
        "armed": True,
        "guided": False,
        "mode": "STABILIZE",
        "system_status": 4,
    }
    assert body["velocity"] == {"x": 0.1, "y": 0.0, "z": 0.0}
    assert body["attitude"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    assert body["gps"] == {
        "fix_status": 0,
        "satellites_visible": 8,
        "latitude": 1.0,
        "longitude": 2.0,
        "altitude": 3.0,
    }
    assert body["statustext"] == [{"severity": 6, "text": "Arming Checks Disabled"}]
    assert body["heartbeat_age_s"] == 0.4


def test_map_snapshot_flattens_occupancy_grid():
    client, fake = make_client()
    fake.set_latest(
        "/map",
        {"info": {"resolution": 1.0, "width": 5, "height": 5}, "data": [-1] * 25},
    )

    body = client.get("/api/map").json()

    assert body["width"] == 5
    assert body["height"] == 5
    assert body["resolution"] == 1.0
    assert len(body["data"]) == 25


def test_map_snapshot_before_any_map_received():
    client, _ = make_client()
    body = client.get("/api/map").json()
    assert body == {"resolution": None, "width": None, "height": None, "data": None, "origin_x": None, "origin_y": None, "robot": None, "age_s": None, "arena": None}


def test_coverage_snapshot_flattens_occupancy_grid():
    client, fake = make_client()
    fake.set_latest(
        "/coverage_grid",
        {"info": {"resolution": 1.0, "width": 3, "height": 3}, "data": [0] * 9},
    )

    body = client.get("/api/coverage").json()

    assert body["width"] == 3
    assert body["height"] == 3
    assert len(body["data"]) == 9


def test_coverage_snapshot_before_any_data_received():
    client, _ = make_client()
    body = client.get("/api/coverage").json()
    assert body == {"resolution": None, "width": None, "height": None, "data": None}


def test_planned_path_flattens_pose_array_to_points():
    client, fake = make_client()
    fake.set_latest(
        "/planned_path",
        {
            "poses": [
                {"pose": {"position": {"x": 1.0, "y": 2.0}}},
                {"pose": {"position": {"x": 3.0, "y": 4.0}}},
            ]
        },
    )

    body = client.get("/api/path").json()

    assert body["points"] == [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]


def test_planned_path_before_any_data_received():
    client, _ = make_client()
    body = client.get("/api/path").json()
    assert body == {"points": []}


def test_frontiers_flattens_marker_array_to_points():
    client, fake = make_client()
    fake.set_latest(
        "/frontiers",
        {
            "markers": [
                {"pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.5}}},
                {"pose": {"position": {"x": 3.0, "y": 4.0, "z": 0.5}}},
            ]
        },
    )

    body = client.get("/api/frontiers").json()

    assert body["points"] == [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]


def test_frontiers_before_any_data_received():
    client, _ = make_client()
    body = client.get("/api/frontiers").json()
    assert body == {"points": []}


def test_telemetry_reflects_mapping_autonomy_navigation_from_telemetry_state():
    """/telemetry/state is a normalized JSON contract from onboard-autonomy
    (see CHECKPOINT/docs/gcs_telemetry_contract.md); RosBridgeClient caches
    it already-parsed, so the fake stores the same shape a real client
    would after JSON-decoding the std_msgs/String payload."""
    client, fake = make_client()
    fake.set_latest(
        "/telemetry/state",
        {
            "autonomy": {
                "state": "SEARCHING_FRONTIER",
                "objective": "Explore unexplored region",
                "target": [4.2, 7.8],
                "next_action": "Navigate to frontier",
            },
            "sensors": {"slam": "ok", "lidar": "ok", "rangefinder": "not_integrated", "camera": "not_integrated"},
            "mapping": {
                "available": True,
                "resolution_m": 0.05,
                "width_cells": 320,
                "height_cells": 320,
                "origin_x": -8.5,
                "origin_y": -1.0,
                "coverage_cell_size_m": 1.0,
                "explored_pct": 63.5,
            },
            "navigation": {
                "target": [4.2, 7.8],
                "frontier_count": 3,
                "candidate_count": 5,
                "blacklisted_count": 1,
                "geofence_breached": False,
            },
        },
    )

    body = client.get("/api/telemetry").json()

    assert body["autonomy"]["state"] == "SEARCHING_FRONTIER"
    assert body["autonomy"]["target"] == [4.2, 7.8]
    assert body["sensors"]["slam"] == "ok"
    assert body["mapping"]["available"] is True
    assert body["mapping"]["explored_pct"] == 63.5
    assert body["navigation"]["frontier_count"] == 3
    assert body["navigation"]["geofence_breached"] is False


def test_telemetry_mapping_defaults_before_any_telemetry_state_received():
    client, _ = make_client()
    body = client.get("/api/telemetry").json()
    assert body["mapping"] == {
        "available": False, "resolution_m": None, "width_cells": None, "height_cells": None,
        "origin_x": None, "origin_y": None, "coverage_cell_size_m": None, "explored_pct": None,
    }
    assert body["autonomy"] == {"state": None, "objective": None, "target": None, "next_action": None}
    assert body["navigation"] == {
        "target": None, "frontier_count": None, "candidate_count": None,
        "blacklisted_count": None, "geofence_breached": None,
    }


def test_survivors_endpoint_reflects_client_state():
    client, fake = make_client()
    fake.set_survivors(
        [
            {"survivor_id": 1, "x": 1.0, "y": 2.0, "confidence": 0.9},
            {"survivor_id": 2, "x": 5.0, "y": 6.0, "confidence": 0.8},
        ]
    )

    body = client.get("/api/survivors").json()

    assert len(body) == 2
    assert body[0]["survivor_id"] == 1


def test_perception_detections_before_any_data_received():
    client, _ = make_client()
    body = client.get("/api/perception/detections").json()
    assert body == {
        "frame_width": None,
        "frame_height": None,
        "timestamp": None,
        "detections": [],
    }


def test_perception_detections_reflects_client_state_with_nested_bbox():
    client, fake = make_client()
    fake.set_perception_detections(
        {
            "frame_width": 1280,
            "frame_height": 720,
            "timestamp": 123.456,
            "detections": [
                {
                    "detection_id": "det-1",
                    "class_name": "person",
                    "confidence": 0.91,
                    "bbox": {"x_min": 10.0, "y_min": 20.0, "x_max": 110.0, "y_max": 220.0},
                    "center_x": 60.0,
                    "center_y": 120.0,
                    "track_id": "t-1",
                    "source": "yolov8n",
                    "model_name": "yolov8n.pt",
                },
                {
                    "detection_id": "det-2",
                    "class_name": "person",
                    "confidence": 0.75,
                    "bbox": {"x_min": 300.0, "y_min": 40.0, "x_max": 380.0, "y_max": 260.0},
                    "center_x": 340.0,
                    "center_y": 150.0,
                    "track_id": "t-2",
                    "source": "yolov8n",
                    "model_name": "yolov8n.pt",
                },
            ],
        }
    )

    body = client.get("/api/perception/detections").json()

    assert body["frame_width"] == 1280
    assert body["frame_height"] == 720
    assert body["timestamp"] == 123.456
    assert len(body["detections"]) == 2
    assert body["detections"][0]["detection_id"] == "det-1"
    assert body["detections"][0]["bbox"] == {
        "x_min": 10.0,
        "y_min": 20.0,
        "x_max": 110.0,
        "y_max": 220.0,
    }
    assert body["detections"][1]["detection_id"] == "det-2"
    assert body["detections"][1]["bbox"]["x_max"] == 380.0


def test_perception_status_before_any_data_received():
    client, _ = make_client()
    body = client.get("/api/perception/status").json()
    assert body == {
        "camera_connected": None,
        "detector_enabled": None,
        "detector_ready": None,
        "detector_backend": None,
        "model_name": None,
        "person_count": None,
        "fps": None,
        "frame_width": None,
        "frame_height": None,
        "last_detection_age_s": None,
    }


def test_perception_status_reflects_client_state_and_ignores_unexpected_keys():
    client, fake = make_client()
    fake.set_perception_status(
        {
            "camera_connected": True,
            "detector_enabled": True,
            "detector_ready": True,
            "detector_backend": "onnxruntime",
            "model_name": "yolov8n.pt",
            "person_count": 2,
            "fps": 12.5,
            "frame_width": 1280,
            "frame_height": 720,
            "last_detection_age_s": 0.2,
            "some_future_field_the_gcs_does_not_know_about": "unexpected",
        }
    )

    resp = client.get("/api/perception/status")

    assert resp.status_code == 200
    body = resp.json()
    assert body["camera_connected"] is True
    assert body["detector_backend"] == "onnxruntime"
    assert body["person_count"] == 2
    assert body["fps"] == 12.5
    assert "some_future_field_the_gcs_does_not_know_about" not in body


def test_camera_status_disconnected_has_no_stream_url():
    client, fake = make_client()
    fake.set_perception_status({"camera_connected": False})

    body = client.get("/api/camera/status").json()

    assert body["connected"] is False
    assert body["stream_url"] is None


def test_camera_status_before_any_data_received():
    client, _ = make_client()
    body = client.get("/api/camera/status").json()
    assert body["connected"] is None
    assert body["stream_url"] is None


def test_camera_status_connected_builds_stream_url_from_settings():
    client, fake = make_client()
    fake.set_perception_status(
        {"camera_connected": True, "frame_width": 1280, "frame_height": 720, "fps": 12.5}
    )

    body = client.get("/api/camera/status").json()

    assert body["connected"] is True
    assert body["stream_url"] == "http://127.0.0.1:8090/stream.mjpg"
    assert body["frame_width"] == 1280
    assert body["frame_height"] == 720
    assert body["fps"] == 12.5


def test_perception_detections_empty_dict_upstream_behaves_same_as_no_data_yet():
    """An empty-but-present `{}` payload (e.g. the topic has been
    advertised/echoed once with a blank body, distinct from never having
    published at all) must still degrade to the same all-defaults
    response as test_perception_detections_before_any_data_received --
    the route's `if not msg` check treats `None` and `{}` identically, so
    this pins that both code paths actually land on the same output."""
    client, fake = make_client()
    fake.set_perception_detections({})

    body = client.get("/api/perception/detections").json()

    assert body == {
        "frame_width": None,
        "frame_height": None,
        "timestamp": None,
        "detections": [],
    }


def test_perception_detections_malformed_upstream_field_type_degrades_to_defaults():
    """A message that HAS arrived but has a field of the wrong type (e.g.
    a non-numeric confidence) must degrade the same way "no data yet"
    does (see the empty-dict test above), not surface as a bare 500 to
    every caller: PerceptionDetectionsResponse(**msg) raises a pydantic
    ValidationError inside the route body, which the route now catches
    and falls back to the schema's own all-defaults response -- see the
    test-engineer finding: unlike ros_client.py's own json.loads() guard
    (which silently drops a malformed WIRE payload before it's ever
    cached), the route layer must separately guard a malformed-but-
    JSON-valid CACHED payload."""
    fake = FakeRosBridgeClient()
    fake.set_perception_detections(
        {
            "frame_width": 640,
            "frame_height": 480,
            "timestamp": 1.0,
            "detections": [{"confidence": "high"}],
        }
    )
    app = create_app(client=fake)
    client = TestClient(app, raise_server_exceptions=False)

    resp = client.get("/api/perception/detections")

    assert resp.status_code == 200
    assert resp.json() == {
        "frame_width": None,
        "frame_height": None,
        "timestamp": None,
        "detections": [],
    }


def test_perception_status_malformed_upstream_field_type_degrades_to_defaults():
    fake = FakeRosBridgeClient()
    fake.set_perception_status({"camera_connected": True, "person_count": "two"})
    app = create_app(client=fake)
    client = TestClient(app, raise_server_exceptions=False)

    resp = client.get("/api/perception/status")

    assert resp.status_code == 200
    assert resp.json() == {
        "camera_connected": None,
        "detector_enabled": None,
        "detector_ready": None,
        "detector_backend": None,
        "model_name": None,
        "person_count": None,
        "fps": None,
        "frame_width": None,
        "frame_height": None,
        "last_detection_age_s": None,
    }


def test_camera_status_malformed_upstream_field_type_degrades_to_defaults():
    fake = FakeRosBridgeClient()
    fake.set_perception_status({"camera_connected": True, "frame_width": "big"})
    app = create_app(client=fake)
    client = TestClient(app, raise_server_exceptions=False)

    resp = client.get("/api/camera/status")

    assert resp.status_code == 200
    assert resp.json() == {
        "connected": None,
        "stream_url": None,
        "frame_width": None,
        "frame_height": None,
        "fps": None,
    }


def test_hover_start_goes_over_the_radio_with_mission_code_not_over_rosbridge():
    client, fake, radio = make_radio_client()
    resp = client.post("/api/mission/start", json={"mission": "hover"})
    assert resp.status_code == 200
    assert resp.json() == {
        "status": "accepted", "command": "start", "mission": "hover", "nonce": 1234, "attempts": 1,
    }
    assert radio.sent == [("start", 1)]
    assert fake.published_commands == []
    assert fake.published_mission_selects == []


def test_motor_test_start_goes_over_the_radio_with_its_own_code():
    client, fake, radio = make_radio_client()
    resp = client.post("/api/mission/start", json={"mission": "motor_test"})
    assert resp.status_code == 200
    assert resp.json()["mission"] == "motor_test"
    assert radio.sent == [("start", 2)]
    assert fake.published_commands == []


def test_abort_goes_over_the_radio_not_over_rosbridge():
    client, fake, radio = make_radio_client()
    resp = client.post("/api/command/abort")
    assert resp.status_code == 200
    assert resp.json()["status"] == "accepted"
    assert resp.json()["command"] == "abort"
    assert radio.sent == [("abort", 0)]
    assert fake.published_commands == []


def test_there_is_no_mission_less_start_route():
    client, _, radio = make_radio_client()
    assert client.post("/api/command/start").status_code in (404, 405)
    assert radio.sent == []


def test_start_without_an_ack_is_a_504_never_success():
    client, _, radio = make_radio_client()
    radio.acked = False
    resp = client.post("/api/mission/start", json={"mission": "hover"})
    assert resp.status_code == 504
    assert "never acknowledged" in resp.json()["detail"]


def test_start_rejected_by_the_jetson_is_a_409_with_the_reason():
    client, _, radio = make_radio_client()
    radio.result = 1
    radio.reason = "FCU_NOT_CONNECTED"
    resp = client.post("/api/mission/start", json={"mission": "hover"})
    assert resp.status_code == 409
    assert resp.json()["detail"] == "Jetson REJECTED START: FCU_NOT_CONNECTED"


def test_abort_without_an_ack_is_a_504():
    client, _, radio = make_radio_client()
    radio.acked = False
    assert client.post("/api/command/abort").status_code == 504


def test_radio_port_unavailable_is_a_503_and_nothing_is_claimed_sent():
    client, _, radio = make_radio_client()
    radio.unavailable = "radio port COM5 is not open (could not open port)"
    for resp in (client.post("/api/mission/start", json={"mission": "hover"}), client.post("/api/command/abort")):
        assert resp.status_code == 503
        assert "not sent" in resp.json()["detail"]
    assert radio.sent == []


def test_unknown_mission_404s_and_sends_nothing():
    client, _, radio = make_radio_client()
    resp = client.post("/api/mission/start", json={"mission": "main_nidar"})
    assert resp.status_code == 404
    assert radio.sent == []


def test_radio_status_route():
    client, _, _ = make_radio_client()
    body = client.get("/api/radio/status").json()
    assert body["port_open"] is True
    assert body["jetson_link_up"] is True
    assert body["jetson_mission_state"] == "idle"


def test_radio_link_is_started_and_stopped_with_the_app():
    radio = FakeRadioLink()
    with TestClient(create_app(client=FakeRosBridgeClient(), radio=radio)):
        pass
    assert not radio.started  # an injected radio is owned by the caller
    owned = FakeRadioLink()
    import app.main as main_module

    original = main_module.RadioLink
    main_module.RadioLink = lambda *a, **k: owned
    try:
        with TestClient(create_app(client=FakeRosBridgeClient())):
            assert owned.started
        assert owned.stopped
    finally:
        main_module.RadioLink = original


def test_get_requests_never_mutate_command_state():
    client, fake = make_client()
    for _ in range(3):
        client.get("/health")
        client.get("/api/telemetry")
        client.get("/api/map")
        client.get("/api/survivors")
    assert fake.published_commands == []


def test_ui_serves_the_frontend_and_wires_the_documented_endpoints():
    """gcs/frontend/ is now a compiled React/Vite SPA, not the old
    single-file prototype -- /ui/ serves a minimal HTML shell (a
    `<div id="root">` plus a hashed, bundled `<script type="module">`)
    and the actual button/fetch logic lives inside that bundle, which
    this Python TestClient has no JS engine to execute. So this test can
    only prove what's still structurally true from the served HTML: the
    mount serves a real build (not an empty/broken directory) at /ui
    specifically. The "calls exactly /api/telemetry,
    /api/command/start, /api/command/abort" guarantee now lives on the
    frontend side -- see gcs/frontend/src/api.test.ts."""
    client, _ = make_client()

    resp = client.get("/ui/")
    assert resp.status_code == 200
    assert "text/html" in resp.headers["content-type"]
    body = resp.text
    assert '<div id="root">' in body
    assert '<script type="module"' in body


def test_no_route_exists_beyond_the_documented_command_surface():
    """This is the structural guarantee the architecture is built around:
    the operator command surface is exactly start/abort because no other
    mutating route is defined -- not because the frontend promises not to
    call one. See docs/REQUIREMENTS.md §6."""
    client, _ = make_client()
    forbidden_paths = (
        "/api/command/waypoint",
        "/api/command/land",
        "/api/command/reset",
        "/api/command/tag_survivor",
        "/api/map/edit",
        "/api/navigation/goto",
    )
    for path in forbidden_paths:
        assert client.post(path).status_code == 404
        assert client.put(path).status_code == 404


def test_list_missions_returns_hover_and_motor_test():
    client, _ = make_client()
    body = client.get("/api/missions").json()
    assert [m["id"] for m in body] == ["hover", "motor_test"]
    assert [m["name"] for m in body] == ["Hover", "Motor Test"]


def test_get_mission_detail_for_hover():
    client, _ = make_client()
    resp = client.get("/api/missions/hover")
    assert resp.status_code == 200
    assert resp.json()["id"] == "hover"


def test_get_mission_detail_unknown_mission_404s():
    client, _ = make_client()
    resp = client.get("/api/missions/bogus")
    assert resp.status_code == 404


def test_flight_test_status_before_any_data_arrives():
    client, _ = make_client()
    body = client.get("/api/flight-test/status").json()
    assert body == {
        "scenario": None,
        "state": None,
        "target_altitude_m": None,
        "current_altitude_m": None,
        "current_position": None,
        "duration_s": None,
        "elapsed_hover_s": None,
        "armed": None,
        "execution_mode": None,
        "detail": None,
        "flight_mode": None,
    }


def test_flight_test_status_reflects_seeded_data():
    client, fake = make_client()
    fake.set_flight_test_status(
        {
            "schema_version": 1,
            "scenario": "hover",
            "state": "hovering",
            "target_altitude_m": 1.0,
            "current_altitude_m": 0.98,
            "current_position": [0.0, 0.0, 0.98],
            "duration_s": 10.0,
            "elapsed_hover_s": 3.2,
            "armed": True,
            "execution_mode": "mock",
            "timestamp": 1234567890.1,
        }
    )

    body = client.get("/api/flight-test/status").json()

    assert body["scenario"] == "hover"
    assert body["state"] == "hovering"
    assert body["target_altitude_m"] == 1.0
    assert body["current_altitude_m"] == 0.98
    assert body["current_position"] == [0.0, 0.0, 0.98]
    assert body["duration_s"] == 10.0
    assert body["elapsed_hover_s"] == 3.2
    assert body["armed"] is True
    assert body["execution_mode"] == "mock"


def test_flight_test_status_malformed_upstream_field_type_degrades_to_defaults():
    fake = FakeRosBridgeClient()
    fake.set_flight_test_status({"state": "hovering", "target_altitude_m": "not-a-number"})
    app = create_app(client=fake)
    client = TestClient(app, raise_server_exceptions=False)

    resp = client.get("/api/flight-test/status")

    assert resp.status_code == 200
    assert resp.json() == {
        "scenario": None,
        "state": None,
        "target_altitude_m": None,
        "current_altitude_m": None,
        "current_position": None,
        "duration_s": None,
        "elapsed_hover_s": None,
        "armed": None,
        "execution_mode": None,
        "detail": None,
        "flight_mode": None,
    }


def test_multi_step_flight_test_status_before_any_data_arrives():
    client, _ = make_client()
    body = client.get("/api/flight-test/multi-step/status").json()
    assert body == {
        "scenario_id": None,
        "state": None,
        "phase": None,
        "current_step_index": None,
        "current_step_action": None,
        "total_steps": None,
        "current_position": None,
        "armed": None,
        "execution_mode": None,
    }


def test_multi_step_flight_test_status_reflects_seeded_data():
    client, fake = make_client()
    fake.set_multi_step_flight_test_status(
        {
            "schema_version": 1,
            "mission_id": "flight_test",
            "scenario_id": "forward_backward_hover",
            "state": "executing",
            "phase": "step",
            "current_step_index": 0,
            "current_step_action": "forward",
            "total_steps": 3,
            "current_position": [1.0, 0.0, 1.0],
            "armed": True,
            "execution_mode": "mock",
            "timestamp": 1234567890.1,
        }
    )

    body = client.get("/api/flight-test/multi-step/status").json()

    assert body["scenario_id"] == "forward_backward_hover"
    assert body["state"] == "executing"
    assert body["phase"] == "step"
    assert body["current_step_index"] == 0
    assert body["current_step_action"] == "forward"
    assert body["total_steps"] == 3
    assert body["current_position"] == [1.0, 0.0, 1.0]
    assert body["armed"] is True
    assert body["execution_mode"] == "mock"


def test_multi_step_flight_test_status_can_report_failed_distinct_from_aborted():
    client, fake = make_client()
    fake.set_multi_step_flight_test_status({"state": "failed", "scenario_id": "forward_backward_hover"})
    body = client.get("/api/flight-test/multi-step/status").json()
    assert body["state"] == "failed"


def test_multi_step_flight_test_status_malformed_upstream_field_type_degrades_to_defaults():
    fake = FakeRosBridgeClient()
    fake.set_multi_step_flight_test_status({"state": "executing", "total_steps": "three"})
    app = create_app(client=fake)
    client = TestClient(app, raise_server_exceptions=False)

    resp = client.get("/api/flight-test/multi-step/status")

    assert resp.status_code == 200
    assert resp.json() == {
        "scenario_id": None,
        "state": None,
        "phase": None,
        "current_step_index": None,
        "current_step_action": None,
        "total_steps": None,
        "current_position": None,
        "armed": None,
        "execution_mode": None,
    }


def test_mission_get_routes_never_send_anything():
    client, fake, radio = make_radio_client()
    for _ in range(3):
        client.get("/api/missions")
        client.get("/api/missions/hover")
        client.get("/api/flight-test/status")
        client.get("/api/radio/status")
    assert radio.sent == []
    assert fake.published_commands == []


def test_main_source_contains_no_forbidden_command_surface_words():
    """Grep-style guard: no new route path or function name in main.py's
    source introduces a throttle/motor/joystick/manual control surface,
    mirroring test_no_route_exists_beyond_the_documented_command_surface's
    structural guarantee at the source-text level."""
    import inspect

    from app import main as main_module

    source = inspect.getsource(main_module)
    for forbidden in ("throttle", "motor", "joystick", "manual"):
        assert forbidden not in source.lower()


def test_new_perception_and_camera_routes_are_get_only():
    """The perception/camera routes added alongside coverage/survivors
    are read-only, same guarantee as
    test_no_route_exists_beyond_the_documented_command_surface above --
    POST/PUT to any of them must 404, since they were never defined as
    mutating routes."""
    client, _ = make_client()
    for path in ("/api/perception/detections", "/api/perception/status", "/api/camera/status"):
        # The path itself exists (only GET is registered), so the
        # wrong-method response is 405 Method Not Allowed, not 404 --
        # either way, no mutation is possible via these paths.
        assert client.post(path).status_code == 405
        assert client.put(path).status_code == 405

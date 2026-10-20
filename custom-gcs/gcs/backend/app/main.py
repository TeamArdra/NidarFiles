"""FastAPI app factory.

`create_app()` with no arguments builds the real thing: connects to
rosbridge at startup per Settings (app/config.py) and disconnects at
shutdown. Tests instead pass `client=<a fake>` to get an app wired to a
test double with no real networking -- see tests/fakes.py.

ROS-optional local development: if Settings.ros_enabled is False
(GCS_ROS_ENABLED=false -- see app/config.py), create_app() wires in
app/ros_client.py's DisabledRosBridgeClient instead of a real
RosBridgeClient, so this app never attempts any ROS/rosbridge
connectivity at all -- lets the FastAPI backend (and, pointed at it, the
frontend) start and be exercised on a machine with no Jetson, no ROS, no
rosbridge (e.g. a plain Windows laptop). Mission/scenario metadata
(GET /api/missions and friends, backed by app/missions.py's static
registry) works identically either way since it never touches ros_client.
ROS-dependent reads degrade to their normal "no data yet" empty/None
shape; ROS-dependent writes (START/ABORT/mission-start/simulation)
503 with a clear "ROS is disabled" detail rather than pretending to have
sent anything -- this mode never fakes a connection or a mission start.
Separately, even when ROS *is* enabled, a failed connect() at startup
(rosbridge unreachable) is caught and logged rather than crashing the
app -- see lifespan() below.

The API surface is intentionally small. In particular: there is no route
that can modify navigation, the map, or survivor tags, and there are
exactly two mutating routes affecting the REAL mission:
POST /api/mission/start (START for a mission chosen from app/missions.py)
and POST /api/command/abort. Both go ONLY over the MicroLR900 command
radio (app/radio_link.py) to the Jetson -- never over Wi-Fi/rosbridge --
and return success only when the Jetson has ACKed AND accepted the
command. There is no mission-less START. Telemetry comes back over the
same radio by default (Settings.telemetry_source "radio",
app/radio_telemetry.py); "rosbridge" keeps the old Wi-Fi path for
development against sim/. See docs/REQUIREMENTS.md §6 and CLAUDE.md
"Important Constraints" §1.

Separately, POST /api/simulation/run and POST /api/simulation/reset
control a self-contained, ROS-topic-isolated simulation (see
onboard-autonomy/nidar_autonomy/simulation_node.py and
CHECKPOINT/CURRENT_STATE.md) -- these publish to /simulation/command,
NEVER to the real /gcs/command, and cannot affect the real mission state
machine or the real Pixhawk under any circumstance (that ROS node never
imports mavros_msgs/flight_command.py at all).
"""

from __future__ import annotations

import logging
import re
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

from .config import Settings, get_settings
from .missions import (
    MISSION_REGISTRY,
    MissionDefinition,
    MissionNotFoundError,
    get_mission,
)
from .radio_link import DisabledRadioLink, RadioLink, RadioUnavailable
from .radio_telemetry import RadioTelemetryClient
from .ros_client import (
    ARENA_GRID_TOPIC,
    BATTERY_TOPIC,
    COVERAGE_GRID_TOPIC,
    FCU_STATE_TOPIC,
    FLIGHT_TEST_STATUS_TOPIC,
    FRONTIERS_TOPIC,
    GPS_TOPIC,
    HEARTBEAT_TOPIC,
    IMU_TOPIC,
    LIDAR_TOPIC,
    MAP_TOPIC,
    MISSION_STATE_TOPIC,
    MULTI_STEP_TEST_STATUS_TOPIC,
    PERCEPTION_DETECTIONS_TOPIC,
    PERCEPTION_STATUS_TOPIC,
    PLANNED_PATH_TOPIC,
    POSE_TOPIC,
    SIMULATION_COVERAGE_GRID_TOPIC,
    SLAM_POSE_TOPIC,
    SIMULATION_MAP_TOPIC,
    SIMULATION_MISSION_STATE_TOPIC,
    SIMULATION_PLANNED_PATH_TOPIC,
    SIMULATION_STATUS_TOPIC,
    SIMULATION_TELEMETRY_STATE_TOPIC,
    TELEMETRY_STATE_TOPIC,
    VELOCITY_TOPIC,
    DisabledRosBridgeClient,
    RosBridgeClient,
)
from .schemas import (
    AttitudeResponse,
    AutonomyStateResponse,
    BatteryResponse,
    CameraStatusResponse,
    CoverageResponse,
    FcuStateResponse,
    FlightTestStatusResponse,
    FrontierPointResponse,
    FrontiersResponse,
    GpsResponse,
    HealthResponse,
    LidarResponse,
    MapResponse,
    MappingStatusResponse,
    MissionResponse,
    MissionStartRequest,
    MultiStepFlightTestStatusResponse,
    NavigationResponse,
    PathPointResponse,
    PathResponse,
    PerceptionDetectionsResponse,
    PerceptionStatusResponse,
    ParamResponse,
    ParamWriteRequest,
    PoseResponse,
    PositionResponse,
    RadioCommandResponse,
    ComponentsResponse,
    RadioStatusResponse,
    SensorsResponse,
    SimulationCommandResponse,
    SimulationStatusResponse,
    StatusTextResponse,
    SurvivorResponse,
    TelemetryResponse,
    VelocityResponse,
)

_FRONTEND_DIR = Path(__file__).resolve().parents[2] / "frontend" / "dist"
_PARAM_NAME = re.compile(r"^[A-Z0-9_]{1,16}$")


def _mission_to_response(mission: MissionDefinition) -> MissionResponse:
    return MissionResponse(id=mission.id, name=mission.name, description=mission.description)


def create_app(
    client: RosBridgeClient | DisabledRosBridgeClient | RadioTelemetryClient | None = None,
    settings: Settings | None = None,
    radio: RadioLink | DisabledRadioLink | None = None,
) -> FastAPI:
    settings = settings or get_settings()
    owns_client = client is None
    owns_radio = radio is None
    if client is not None:
        ros_client = client
    elif settings.telemetry_source == "radio":
        # Fed by the RadioLink built below. With the radio disabled it is
        # never fed and reports "no data" -- never a faked connection.
        ros_client = RadioTelemetryClient()
    elif settings.ros_enabled:
        ros_client = RosBridgeClient(
            settings.rosbridge_host, settings.rosbridge_port, settings.connect_timeout_s
        )
    else:
        # GCS_ROS_ENABLED=false -- ROS-optional local-development mode,
        # see this module's docstring. Never falls back to this silently:
        # it's exactly and only settings.ros_enabled being False that
        # selects it.
        ros_client = DisabledRosBridgeClient()
    if radio is None:
        radio = (
            RadioLink(
                settings.radio_port,
                settings.radio_baud,
                telemetry=ros_client if isinstance(ros_client, RadioTelemetryClient) else None,
            )
            if settings.radio_enabled
            else DisabledRadioLink()
        )

    def ros_status() -> str:
        """"connected" / "disabled" / "unavailable" -- see
        HealthResponse.ros_status's docstring in app/schemas.py. Derived
        from settings.ros_enabled, never from the concrete ros_client
        type, so this reports correctly whether ros_client is a real
        RosBridgeClient, a DisabledRosBridgeClient, or a test double
        (tests/fakes.py's FakeRosBridgeClient) that has no concept of
        "disabled" at all. Radio telemetry uses no ROS link: "disabled"."""
        if not settings.ros_enabled or settings.telemetry_source == "radio":
            return "disabled"
        return "connected" if ros_client.is_connected else "unavailable"

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        if owns_radio:
            radio.start()
        if owns_client:
            try:
                ros_client.connect()
            except Exception as exc:
                # rosbridge unreachable (no Jetson, rosbridge_server not
                # running, wrong host/port, network down, ...) -- log and
                # keep running disconnected rather than crashing FastAPI
                # startup. Every route already degrades gracefully when
                # ros_client.is_connected is False (see e.g. /api/telemetry
                # above), and GET /health's ros_status reports
                # "unavailable" so this is never silently upgraded to
                # "connected". When settings.ros_enabled is False,
                # ros_client.connect() is DisabledRosBridgeClient's
                # deliberate no-op and never raises, so this branch is
                # specific to the ROS-enabled-but-unreachable case.
                logging.getLogger(__name__).warning(
                    "ROS connection failed (rosbridge unavailable at %s:%s): %s "
                    "-- continuing without ROS connectivity",
                    settings.rosbridge_host, settings.rosbridge_port, exc,
                )
        try:
            yield
        finally:
            if owns_client:
                ros_client.disconnect()
            if owns_radio:
                radio.stop()

    app = FastAPI(
        title="NIDAR AirMouse GCS Backend",
        description=(
            "The entire operator command surface affecting the real mission "
            "is exactly two endpoints, both sent over the MicroLR900 command "
            "radio: POST /api/mission/start (START for the selected mission) "
            "and POST /api/command/abort. No other route can affect the "
            "mission -- see docs/REQUIREMENTS.md §6."
        ),
        lifespan=lifespan,
    )

    # Local-only dev convenience so a frontend on a different port (e.g. a
    # Vite dev server) can call this API. Never exposed beyond the local
    # link per the competition's no-external-network constraint.
    app.add_middleware(
        CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
    )

    @app.get("/health", response_model=HealthResponse, tags=["status"])
    def health() -> HealthResponse:
        return HealthResponse(
            connected=ros_client.is_connected,
            ros_status=ros_status(),
            telemetry_source=settings.telemetry_source,
            setup_enabled=settings.setup_enabled,
            rosbridge_host=settings.rosbridge_host,
            rosbridge_port=settings.rosbridge_port,
        )

    @app.get("/api/telemetry", response_model=TelemetryResponse, tags=["telemetry"])
    def telemetry() -> TelemetryResponse:
        state_msg = ros_client.latest(MISSION_STATE_TOPIC) or {}
        battery_msg = ros_client.latest(BATTERY_TOPIC) or {}
        pose_msg = ros_client.latest(POSE_TOPIC) or {}
        fcu_msg = ros_client.latest(FCU_STATE_TOPIC) or {}
        velocity_msg = ros_client.latest(VELOCITY_TOPIC) or {}
        gps_msg = ros_client.latest(GPS_TOPIC) or {}
        imu_msg = ros_client.latest(IMU_TOPIC) or {}
        position = (pose_msg.get("pose") or {}).get("position")
        linear_velocity = (velocity_msg.get("twist") or {}).get("linear")
        orientation = imu_msg.get("orientation")
        gps_status = gps_msg.get("status") or {}

        # Mapping/exploration/planning/autonomy state -- sourced entirely
        # from /telemetry/state (onboard-autonomy's normalized contract),
        # already parsed to a dict by RosBridgeClient. Missing/not-yet-
        # published fields fall back to each response model's own
        # defaults (never fabricated) rather than raising.
        telemetry_state = ros_client.latest(TELEMETRY_STATE_TOPIC) or {}
        autonomy = telemetry_state.get("autonomy") or {}
        sensors = telemetry_state.get("sensors") or {}
        mapping = telemetry_state.get("mapping") or {}
        navigation = telemetry_state.get("navigation") or {}

        return TelemetryResponse(
            connected=ros_client.is_connected,
            mission_state=state_msg.get("data"),
            fcu=FcuStateResponse(
                connected=fcu_msg.get("connected"),
                armed=fcu_msg.get("armed"),
                guided=fcu_msg.get("guided"),
                mode=fcu_msg.get("mode"),
                system_status=fcu_msg.get("system_status"),
            ),
            battery=BatteryResponse(
                voltage=battery_msg.get("voltage"),
                current=battery_msg.get("current"),
                percentage=battery_msg.get("percentage"),
            ),
            pose=PoseResponse(position=PositionResponse(**position) if position else None),
            velocity=VelocityResponse(**linear_velocity) if linear_velocity else None,
            attitude=AttitudeResponse(**orientation) if orientation else None,
            gps=GpsResponse(
                fix_status=gps_status.get("status"),
                satellites_visible=gps_msg.get("satellites_visible"),
                latitude=gps_msg.get("latitude"),
                longitude=gps_msg.get("longitude"),
                altitude=gps_msg.get("altitude"),
            )
            if gps_msg
            else None,
            statustext=[
                StatusTextResponse(severity=m.get("severity"), text=m.get("text"))
                for m in ros_client.statustext_history()
            ],
            heartbeat_age_s=ros_client.age_s(HEARTBEAT_TOPIC),
            autonomy=AutonomyStateResponse(
                state=autonomy.get("state"),
                objective=autonomy.get("objective"),
                target=autonomy.get("target"),
                next_action=autonomy.get("next_action"),
            ),
            sensors=SensorsResponse(
                slam=sensors.get("slam"),
                lidar=sensors.get("lidar"),
                rangefinder=sensors.get("rangefinder"),
                camera=sensors.get("camera"),
            ),
            mapping=MappingStatusResponse(
                available=bool(mapping.get("available", False)),
                resolution_m=mapping.get("resolution_m"),
                width_cells=mapping.get("width_cells"),
                height_cells=mapping.get("height_cells"),
                origin_x=mapping.get("origin_x"),
                origin_y=mapping.get("origin_y"),
                coverage_cell_size_m=mapping.get("coverage_cell_size_m"),
                explored_pct=mapping.get("explored_pct"),
            ),
            navigation=NavigationResponse(
                target=navigation.get("target"),
                frontier_count=navigation.get("frontier_count"),
                candidate_count=navigation.get("candidate_count"),
                blacklisted_count=navigation.get("blacklisted_count"),
                geofence_breached=navigation.get("geofence_breached"),
            ),
        )

    @app.get("/api/lidar", response_model=LidarResponse, tags=["telemetry"])
    def lidar_scan() -> LidarResponse:
        scan = ros_client.latest(LIDAR_TOPIC)
        if not scan:
            return LidarResponse()
        return LidarResponse(available=True, **scan)

    @app.get("/api/map", response_model=MapResponse, tags=["map"])
    def map_snapshot() -> MapResponse:
        msg = ros_client.latest(MAP_TOPIC)
        if msg is None:
            return MapResponse()
        info = msg.get("info", {})
        origin = (info.get("origin") or {}).get("position") or {}
        robot = ros_client.latest(SLAM_POSE_TOPIC)
        return MapResponse(
            resolution=info.get("resolution"),
            width=info.get("width"),
            height=info.get("height"),
            data=msg.get("data"),
            origin_x=origin.get("x"),
            origin_y=origin.get("y"),
            robot=robot,
            age_s=msg.get("age_s"),
            arena=ros_client.latest(ARENA_GRID_TOPIC),
        )

    @app.get("/api/coverage", response_model=CoverageResponse, tags=["map"])
    def coverage_snapshot() -> CoverageResponse:
        msg = ros_client.latest(COVERAGE_GRID_TOPIC)
        if msg is None:
            return CoverageResponse()
        info = msg.get("info", {})
        return CoverageResponse(
            resolution=info.get("resolution"),
            width=info.get("width"),
            height=info.get("height"),
            data=msg.get("data"),
        )

    @app.get("/api/path", response_model=PathResponse, tags=["map"])
    def planned_path() -> PathResponse:
        msg = ros_client.latest(PLANNED_PATH_TOPIC)
        if msg is None:
            return PathResponse()
        points = [
            PathPointResponse(
                x=(pose.get("pose") or {}).get("position", {}).get("x", 0.0),
                y=(pose.get("pose") or {}).get("position", {}).get("y", 0.0),
            )
            for pose in msg.get("poses", [])
        ]
        return PathResponse(points=points)

    @app.get("/api/frontiers", response_model=FrontiersResponse, tags=["map"])
    def frontiers() -> FrontiersResponse:
        msg = ros_client.latest(FRONTIERS_TOPIC)
        if msg is None:
            return FrontiersResponse()
        # visualization_msgs/Marker.pose is a plain geometry_msgs/Pose, one
        # level of nesting less than nav_msgs/Path's PoseStamped poses (see
        # planned_path() above) -- position is at marker["pose"]["position"],
        # not marker["pose"]["pose"]["position"].
        points = [
            FrontierPointResponse(
                x=(marker.get("pose") or {}).get("position", {}).get("x", 0.0),
                y=(marker.get("pose") or {}).get("position", {}).get("y", 0.0),
            )
            for marker in msg.get("markers", [])
        ]
        return FrontiersResponse(points=points)

    @app.get("/api/survivors", response_model=list[SurvivorResponse], tags=["survivors"])
    def survivors() -> list[SurvivorResponse]:
        return [SurvivorResponse(**s) for s in ros_client.survivors()]

    # -- Perception pipeline (Jetson-side, development-only pretrained
    # person-detector) -- read-only, and architecturally distinct from
    # /api/survivors above: raw/unconfirmed/image-space vs.
    # confirmed/localized/world-coordinate. See
    # PerceptionDetectionsResponse/DetectionResponse docstrings in
    # app/schemas.py. Video bytes never flow through this backend -- see
    # docs/DECISIONS.md D-6 -- /api/camera/status only reports health/
    # metadata plus the stream URL the frontend fetches directly.

    @app.get(
        "/api/perception/detections",
        response_model=PerceptionDetectionsResponse,
        tags=["perception"],
    )
    def perception_detections() -> PerceptionDetectionsResponse:
        msg = ros_client.latest(PERCEPTION_DETECTIONS_TOPIC)
        if not msg:
            return PerceptionDetectionsResponse()
        try:
            return PerceptionDetectionsResponse(**msg)
        except ValidationError:
            # A cached-but-malformed upstream payload (wrong field type from
            # a buggy/malfunctioning publisher) must degrade the same way
            # "no data yet" does, not surface as a bare 500 to every caller.
            return PerceptionDetectionsResponse()

    @app.get(
        "/api/perception/status",
        response_model=PerceptionStatusResponse,
        tags=["perception"],
    )
    def perception_status() -> PerceptionStatusResponse:
        msg = ros_client.latest(PERCEPTION_STATUS_TOPIC)
        if not msg:
            return PerceptionStatusResponse()
        allowed = set(PerceptionStatusResponse.model_fields)
        try:
            return PerceptionStatusResponse(**{k: v for k, v in msg.items() if k in allowed})
        except ValidationError:
            # See perception_detections above -- same degrade-not-500 rule.
            return PerceptionStatusResponse()

    @app.get("/api/camera/status", response_model=CameraStatusResponse, tags=["camera"])
    def camera_status() -> CameraStatusResponse:
        msg = ros_client.latest(PERCEPTION_STATUS_TOPIC)
        connected = bool(msg.get("camera_connected")) if msg else None
        try:
            return CameraStatusResponse(
                connected=connected,
                stream_url=settings.camera_stream_url() if connected else None,
                frame_width=(msg.get("frame_width") if msg else None),
                frame_height=(msg.get("frame_height") if msg else None),
                fps=(msg.get("fps") if msg else None),
            )
        except ValidationError:
            # See perception_detections above -- same degrade-not-500 rule.
            return CameraStatusResponse()

    def send_over_radio(command: str, mission: MissionDefinition | None) -> RadioCommandResponse:
        """START/ABORT go only over the command radio. Success is reported
        only when the Jetson ACKed and accepted the command."""
        label = command.upper()
        try:
            result = radio.send_command(command, mission.radio_code if mission else 0)
        except RadioUnavailable as exc:
            raise HTTPException(status_code=503, detail=f"{exc} -- {label} not sent") from exc
        if not result.acked:
            raise HTTPException(
                status_code=504,
                detail=(
                    f"{label} sent {result.attempts}x over the radio but the Jetson never "
                    "acknowledged it -- radio link down or Jetson radio node not running"
                ),
            )
        if not result.accepted:
            raise HTTPException(status_code=409, detail=f"Jetson REJECTED {label}: {result.reason}")
        return RadioCommandResponse(
            status="accepted",
            command=command,
            mission=mission.id if mission else None,
            nonce=result.nonce,
            attempts=result.attempts,
        )

    @app.post("/api/command/abort", response_model=RadioCommandResponse, tags=["command"])
    def abort_mission() -> RadioCommandResponse:
        return send_over_radio("abort", None)

    @app.get("/api/radio/status", response_model=RadioStatusResponse, tags=["command"])
    def radio_status() -> RadioStatusResponse:
        return RadioStatusResponse(**radio.status())

    @app.get("/api/components", response_model=ComponentsResponse, tags=["telemetry"])
    def components() -> ComponentsResponse:
        if not isinstance(ros_client, RadioTelemetryClient):
            return ComponentsResponse()
        return ComponentsResponse(available=True, components=ros_client.components())

    # -- Bench Setup page (GCS_SETUP_ENABLED only) ----------------------------
    #
    # Read/write Pixhawk parameters over the radio, through the Jetson.
    # Deliberately NOT part of the operator command surface: these routes
    # do not exist unless the backend was started for bench setup, and the
    # Jetson itself refuses writes unless started with --setup, while the
    # vehicle is armed, or while a mission runs.

    if settings.setup_enabled:

        def check_param_name(name: str) -> str:
            if not _PARAM_NAME.match(name):
                raise HTTPException(status_code=422, detail=f"invalid parameter name {name!r}")
            return name

        def param_reply(result, action: str) -> ParamResponse:
            if not result.replied:
                raise HTTPException(
                    status_code=504,
                    detail=f"{action} {result.name}: no reply from the Jetson after "
                    f"{result.attempts} radio attempts",
                )
            if result.error is not None or result.value is None:
                raise HTTPException(status_code=409, detail=f"{action} {result.name} refused: {result.error}")
            return ParamResponse(name=result.name, value=result.value, attempts=result.attempts)

        @app.get("/api/setup/param/{name}", response_model=ParamResponse, tags=["setup"])
        def read_param(name: str) -> ParamResponse:
            check_param_name(name)
            try:
                return param_reply(radio.read_param(name), "read")
            except RadioUnavailable as exc:
                raise HTTPException(status_code=503, detail=str(exc)) from exc

        @app.post("/api/setup/param", response_model=ParamResponse, tags=["setup"])
        def write_param(body: ParamWriteRequest) -> ParamResponse:
            check_param_name(body.name)
            try:
                return param_reply(radio.set_param(body.name, body.value), "write")
            except RadioUnavailable as exc:
                raise HTTPException(status_code=503, detail=str(exc)) from exc

    # -- Missions -------------------------------------------------------------
    #
    # app/missions.py's static registry is the only source of truth for
    # which missions exist (the Mission dropdown). Selecting one is not an
    # operator action; START for it is POST /api/mission/start below.

    @app.get("/api/missions", response_model=list[MissionResponse], tags=["missions"])
    def list_missions() -> list[MissionResponse]:
        return [_mission_to_response(m) for m in MISSION_REGISTRY]

    @app.get("/api/missions/{mission_id}", response_model=MissionResponse, tags=["missions"])
    def get_mission_detail(mission_id: str) -> MissionResponse:
        try:
            return _mission_to_response(get_mission(mission_id))
        except MissionNotFoundError:
            raise HTTPException(status_code=404, detail=f"unknown mission: {mission_id!r}")

    @app.get("/api/flight-test/status", response_model=FlightTestStatusResponse, tags=["flight_test"])
    def flight_test_status() -> FlightTestStatusResponse:
        msg = ros_client.latest(FLIGHT_TEST_STATUS_TOPIC)
        if not msg:
            return FlightTestStatusResponse()
        allowed = set(FlightTestStatusResponse.model_fields)
        try:
            return FlightTestStatusResponse(**{k: v for k, v in msg.items() if k in allowed})
        except ValidationError:
            # Malformed cached data degrades to defaults, same
            # degrade-not-500 rule as the perception routes above.
            return FlightTestStatusResponse()

    @app.get(
        "/api/flight-test/multi-step/status",
        response_model=MultiStepFlightTestStatusResponse,
        tags=["flight_test"],
    )
    def multi_step_flight_test_status() -> MultiStepFlightTestStatusResponse:
        msg = ros_client.latest(MULTI_STEP_TEST_STATUS_TOPIC)
        if not msg:
            return MultiStepFlightTestStatusResponse()
        allowed = set(MultiStepFlightTestStatusResponse.model_fields)
        try:
            return MultiStepFlightTestStatusResponse(**{k: v for k, v in msg.items() if k in allowed})
        except ValidationError:
            # Same degrade-not-500 rule as flight_test_status above.
            return MultiStepFlightTestStatusResponse()

    @app.post("/api/mission/start", response_model=RadioCommandResponse, tags=["missions"])
    def start_mission_by_id(body: MissionStartRequest) -> RadioCommandResponse:
        try:
            mission = get_mission(body.mission)
        except MissionNotFoundError:
            raise HTTPException(status_code=404, detail=f"unknown mission: {body.mission!r}")
        return send_over_radio("start", mission)

    # -- Simulation control surface -----------------------------------------
    #
    # Entirely separate from the real command/telemetry surface above:
    # different publish method (publish_simulation_command, never
    # publish_command), different topic (/simulation/command, never
    # /gcs/command), different response type (SimulationStatusResponse,
    # never TelemetryResponse). See CHECKPOINT/CURRENT_STATE.md and
    # onboard-autonomy/nidar_autonomy/simulation_node.py.

    @app.post("/api/simulation/run", response_model=SimulationCommandResponse, tags=["simulation"])
    def run_simulation() -> SimulationCommandResponse:
        try:
            ros_client.publish_simulation_command("run")
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=f"{exc} -- command not sent") from exc
        return SimulationCommandResponse(status="sent", command="run")

    @app.post("/api/simulation/reset", response_model=SimulationCommandResponse, tags=["simulation"])
    def reset_simulation() -> SimulationCommandResponse:
        try:
            ros_client.publish_simulation_command("reset")
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=f"{exc} -- command not sent") from exc
        return SimulationCommandResponse(status="sent", command="reset")

    @app.get("/api/simulation/status", response_model=SimulationStatusResponse, tags=["simulation"])
    def simulation_status() -> SimulationStatusResponse:
        contract = ros_client.latest(SIMULATION_TELEMETRY_STATE_TOPIC) or {}
        status_msg = ros_client.latest(SIMULATION_STATUS_TOPIC) or {}
        mission_state_msg = ros_client.latest(SIMULATION_MISSION_STATE_TOPIC) or {}
        autonomy = contract.get("autonomy") or {}
        sensors = contract.get("sensors") or {}
        mapping = contract.get("mapping") or {}
        navigation = contract.get("navigation") or {}
        position = contract.get("position") or {}

        return SimulationStatusResponse(
            status=status_msg.get("data", "idle"),
            mission_state=mission_state_msg.get("data", "idle"),
            step=contract.get("simulation_step", 0),
            elapsed_sim_seconds=(contract.get("mission") or {}).get("elapsed_sec", 0.0) or 0.0,
            pose=PositionResponse(x=position["x"], y=position["y"], z=position["z"])
            if position.get("x") is not None
            else None,
            autonomy=AutonomyStateResponse(
                state=autonomy.get("state"),
                objective=autonomy.get("objective"),
                target=autonomy.get("target"),
                next_action=autonomy.get("next_action"),
            ),
            sensors=SensorsResponse(
                slam=sensors.get("slam"),
                lidar=sensors.get("lidar"),
                rangefinder=sensors.get("rangefinder"),
                camera=sensors.get("camera"),
            ),
            mapping=MappingStatusResponse(
                available=bool(mapping.get("available", False)),
                resolution_m=mapping.get("resolution_m"),
                width_cells=mapping.get("width_cells"),
                height_cells=mapping.get("height_cells"),
                origin_x=mapping.get("origin_x"),
                origin_y=mapping.get("origin_y"),
                coverage_cell_size_m=mapping.get("coverage_cell_size_m"),
                explored_pct=mapping.get("explored_pct"),
            ),
            navigation=NavigationResponse(
                target=navigation.get("target"),
                frontier_count=navigation.get("frontier_count"),
                candidate_count=navigation.get("candidate_count"),
                blacklisted_count=navigation.get("blacklisted_count"),
                geofence_breached=navigation.get("geofence_breached"),
            ),
            map_known_pct=mapping.get("explored_pct") or 0.0,
            coverage_search_pct=contract.get("coverage_search_pct", 0.0) or 0.0,
            error=contract.get("error"),
        )

    @app.get("/api/simulation/map", response_model=MapResponse, tags=["simulation"])
    def simulation_map() -> MapResponse:
        msg = ros_client.latest(SIMULATION_MAP_TOPIC)
        if msg is None:
            return MapResponse()
        info = msg.get("info", {})
        return MapResponse(
            resolution=info.get("resolution"),
            width=info.get("width"),
            height=info.get("height"),
            data=msg.get("data"),
        )

    @app.get("/api/simulation/coverage", response_model=CoverageResponse, tags=["simulation"])
    def simulation_coverage() -> CoverageResponse:
        msg = ros_client.latest(SIMULATION_COVERAGE_GRID_TOPIC)
        if msg is None:
            return CoverageResponse()
        info = msg.get("info", {})
        return CoverageResponse(
            resolution=info.get("resolution"),
            width=info.get("width"),
            height=info.get("height"),
            data=msg.get("data"),
        )

    @app.get("/api/simulation/path", response_model=PathResponse, tags=["simulation"])
    def simulation_path() -> PathResponse:
        msg = ros_client.latest(SIMULATION_PLANNED_PATH_TOPIC)
        if msg is None:
            return PathResponse()
        points = [
            PathPointResponse(
                x=(pose.get("pose") or {}).get("position", {}).get("x", 0.0),
                y=(pose.get("pose") or {}).get("position", {}).get("y", 0.0),
            )
            for pose in msg.get("poses", [])
        ]
        return PathResponse(points=points)

    # Static operator UI (React, built via `npm run build` in
    # gcs/frontend/ -- see gcs/frontend/README.md), mounted at /ui (not
    # /) so it can never intercept an unmatched API path -- StaticFiles
    # returns 405 for non-GET/HEAD requests to anything under its mount,
    # which would otherwise shadow
    # test_no_route_exists_beyond_the_documented_command_surface's 404
    # expectation for forbidden paths if mounted at "/". Serves the
    # *build output* (frontend/dist/), not frontend/ source, and is
    # simply absent (mount skipped) if dist/ hasn't been built yet --
    # /ui/ 404s rather than serving raw source or crashing. Calls this
    # same API over HTTP, nothing else -- see gcs/frontend/.
    if _FRONTEND_DIR.is_dir():
        app.mount("/ui", StaticFiles(directory=str(_FRONTEND_DIR), html=True), name="frontend")

    return app


app = create_app()

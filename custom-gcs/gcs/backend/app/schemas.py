"""API response shapes. Deliberately re-shaped/flattened from the raw ROS
message dicts (see docs/DATA_MODELS.md) into plain, frontend-friendly
JSON -- the frontend should never need to know what a PoseStamped or an
OccupancyGrid is."""

from __future__ import annotations

from pydantic import BaseModel


class HealthResponse(BaseModel):
    connected: bool
    # "connected": ROS enabled and rosbridge reachable right now.
    # "disabled": GCS_ROS_ENABLED=false -- this backend was started in
    #   ROS-optional local-development mode (see app/config.py,
    #   app/ros_client.py's DisabledRosBridgeClient) and never attempts
    #   any ROS/rosbridge connectivity at all.
    # "unavailable": ROS enabled but rosbridge is not currently reachable
    #   (no Jetson, rosbridge_server not running, wrong host/port, network
    #   down, ...). Deliberately distinct from "disabled" so an
    #   operator/developer can tell "we chose not to connect" apart from
    #   "we tried and failed" at a glance.
    ros_status: str
    # "radio": telemetry over the MicroLR900 radio (app/radio_telemetry.py),
    # `connected` = Jetson heartbeats arriving; ros_status is then
    # "disabled". "rosbridge": the Wi-Fi path, as described above.
    telemetry_source: str = "rosbridge"
    # True only when started for bench setup (GCS_SETUP_ENABLED): the
    # frontend then shows the Setup section.
    setup_enabled: bool = False
    rosbridge_host: str
    rosbridge_port: int


class BatteryResponse(BaseModel):
    voltage: float | None = None
    current: float | None = None
    percentage: float | None = None


class PositionResponse(BaseModel):
    x: float
    y: float
    z: float


class PoseResponse(BaseModel):
    position: PositionResponse | None = None


class VelocityResponse(BaseModel):
    x: float
    y: float
    z: float


class AttitudeResponse(BaseModel):
    """Orientation quaternion straight from /mavros/imu/data -- left as a
    quaternion rather than converted to Euler angles here, so the backend
    doesn't silently pick a convention the frontend didn't ask for."""

    x: float
    y: float
    z: float
    w: float


class GpsResponse(BaseModel):
    fix_status: int | None = None
    satellites_visible: int | None = None
    latitude: float | None = None
    longitude: float | None = None
    altitude: float | None = None


class StatusTextResponse(BaseModel):
    severity: int
    text: str


class FcuStateResponse(BaseModel):
    """Straight from /mavros/state. `connected` here is the FCU<->MAVROS
    link (distinct from TelemetryResponse.connected, which is the GCS<->
    rosbridge link) -- both can fail independently and the operator needs
    to tell them apart."""

    connected: bool | None = None
    armed: bool | None = None
    guided: bool | None = None
    mode: str | None = None
    system_status: int | None = None


class AutonomyStateResponse(BaseModel):
    """Structured "Active Thinking" panel state -- fixed vocabulary only,
    never free-form/LLM-generated text. See
    CHECKPOINT/docs/gcs_telemetry_contract.md and
    onboard-autonomy/nidar_autonomy/telemetry_contract.py."""

    state: str | None = None
    objective: str | None = None
    target: list[float] | None = None
    next_action: str | None = None


class SensorsResponse(BaseModel):
    slam: str | None = None
    lidar: str | None = None
    rangefinder: str | None = None
    camera: str | None = None


class MappingStatusResponse(BaseModel):
    """Lightweight mapping *summary* -- the full occupancy grid is served
    separately via GET /api/map, not duplicated here."""

    available: bool = False
    resolution_m: float | None = None
    width_cells: int | None = None
    height_cells: int | None = None
    origin_x: float | None = None
    origin_y: float | None = None
    coverage_cell_size_m: float | None = None
    explored_pct: float | None = None


class NavigationResponse(BaseModel):
    target: list[float] | None = None
    frontier_count: int | None = None
    candidate_count: int | None = None
    blacklisted_count: int | None = None
    geofence_breached: bool | None = None


class TelemetryResponse(BaseModel):
    connected: bool
    mission_state: str | None = None
    fcu: FcuStateResponse = FcuStateResponse()
    battery: BatteryResponse
    pose: PoseResponse
    velocity: VelocityResponse | None = None
    attitude: AttitudeResponse | None = None
    gps: GpsResponse | None = None
    statustext: list[StatusTextResponse] = []
    heartbeat_age_s: float | None = None
    # Mapping/exploration/planning/autonomy state, sourced from
    # /telemetry/state (onboard-autonomy's normalized contract -- see
    # CHECKPOINT/docs/gcs_telemetry_contract.md). Independent of the
    # fields above, which stay sourced directly from raw mavros topics
    # exactly as before this was added -- this is additive, not a
    # replacement of an already-verified source of truth (Checkpoint 1-4).
    autonomy: AutonomyStateResponse = AutonomyStateResponse()
    sensors: SensorsResponse = SensorsResponse()
    mapping: MappingStatusResponse = MappingStatusResponse()
    navigation: NavigationResponse = NavigationResponse()


class RobotPoseResponse(BaseModel):
    """Position in the map frame (metres) and heading (degrees,
    counter-clockwise from +x) -- from SLAM over the radio."""

    x: float
    y: float
    yaw_deg: float


class ArenaGridResponse(BaseModel):
    """The map as the organiser's 1 m grid (radio ARENA_GRID; see
    radio_protocol.py for the frame). Edge values: 0 unknown, 1 open, 2 wall.
    v_edges[r][k]: west edge of column col0 + k in row row0 + r (k = cols:
    east edge of the last column). h_edges[k][c]: south edge of row row0 + k
    in column col0 + c (k = rows: north edge of the last row)."""

    theta_deg: float
    offset_u: float
    offset_v: float
    entry_u: int
    entry_v: int
    col0: int
    row0: int
    cols: int
    rows: int
    v_edges: list[list[int]]
    h_edges: list[list[int]]
    age_s: float | None = None


class MapResponse(BaseModel):
    resolution: float | None = None
    width: int | None = None
    height: int | None = None
    data: list[int] | None = None
    # Where cell (0,0)'s corner is in the map frame (metres), and the SLAM
    # pose of the LiDAR/drone -- only with the radio SLAM map (start_jetson.sh --map).
    origin_x: float | None = None
    origin_y: float | None = None
    robot: RobotPoseResponse | None = None
    age_s: float | None = None
    arena: ArenaGridResponse | None = None


class CoverageResponse(BaseModel):
    """Same shape as MapResponse -- the coverage grid is also a
    nav_msgs/OccupancyGrid, just with different cell semantics (-1=unknown,
    0=free-not-yet-searched, 100=searched). See
    CHECKPOINT/docs/gcs_telemetry_contract.md."""

    resolution: float | None = None
    width: int | None = None
    height: int | None = None
    data: list[int] | None = None


class PathPointResponse(BaseModel):
    x: float
    y: float


class PathResponse(BaseModel):
    """Flattened from nav_msgs/Path -- the frontend only needs the
    waypoint list, not the full PoseStamped structure per point."""

    points: list[PathPointResponse] = []


class FrontierPointResponse(BaseModel):
    x: float
    y: float


class FrontiersResponse(BaseModel):
    """Flattened from visualization_msgs/MarkerArray on /frontiers -- the
    frontend only needs the candidate point list, not the full Marker
    structure per point."""

    points: list[FrontierPointResponse] = []


class SurvivorResponse(BaseModel):
    survivor_id: int
    x: float
    y: float
    confidence: float


class BBoxResponse(BaseModel):
    """Image-space bounding box, in the source frame's own pixel
    coordinates -- not world coordinates, and not resolved against the
    grid SurvivorResponse uses."""

    x_min: float | None = None
    y_min: float | None = None
    x_max: float | None = None
    y_max: float | None = None


class DetectionResponse(BaseModel):
    """A single raw, unconfirmed detection from the development-only
    pretrained person-detector running on the Jetson perception pipeline
    (see /perception/detections in app/ros_client.py). Deliberately a
    SEPARATE type from SurvivorResponse: this is image-space and
    unconfirmed (no localization, no operator/mission confirmation),
    where SurvivorResponse is world-coordinate and confirmed. Never merge
    or repurpose one as the other."""

    detection_id: str | None = None
    class_name: str | None = None
    confidence: float | None = None
    bbox: BBoxResponse = BBoxResponse()
    center_x: float | None = None
    center_y: float | None = None
    track_id: str | None = None
    source: str | None = None
    model_name: str | None = None


class PerceptionDetectionsResponse(BaseModel):
    """Snapshot of /perception/detections -- see DetectionResponse's
    docstring for why this is architecturally distinct from
    /api/survivors."""

    frame_width: int | None = None
    frame_height: int | None = None
    timestamp: float | None = None
    detections: list[DetectionResponse] = []


class PerceptionStatusResponse(BaseModel):
    """Snapshot of /perception/status -- health/metadata for the Jetson
    perception pipeline (camera + detector), not detection data itself."""

    camera_connected: bool | None = None
    detector_enabled: bool | None = None
    detector_ready: bool | None = None
    detector_backend: str | None = None
    model_name: str | None = None
    person_count: int | None = None
    fps: float | None = None
    frame_width: int | None = None
    frame_height: int | None = None
    last_detection_age_s: float | None = None


class CameraStatusResponse(BaseModel):
    """Camera health/metadata plus the MJPEG stream URL the frontend's
    <video>/<img> element should point at directly -- video bytes
    themselves never flow through this backend or rosbridge, see
    docs/DECISIONS.md D-6 and docs/DATA_MODELS.md §7."""

    connected: bool | None = None
    stream_url: str | None = None
    frame_width: int | None = None
    frame_height: int | None = None
    fps: float | None = None


class CommandResponse(BaseModel):
    status: str
    command: str


class MissionResponse(BaseModel):
    id: str
    name: str
    description: str


class FlightTestStatusResponse(BaseModel):
    """Snapshot of /flight_test/status. Published by the real hover mission
    on the Jetson (missions/hover/mission.py) with execution_mode "real".
    `execution_mode` is shown as-is so a real run is never confused with
    a mock/simulated one."""

    scenario: str | None = None
    state: str | None = None
    target_altitude_m: float | None = None
    current_altitude_m: float | None = None
    current_position: list[float] | None = None
    duration_s: float | None = None
    elapsed_hover_s: float | None = None
    armed: bool | None = None
    execution_mode: str | None = None
    # Added for the real hover mission (missions/hover on the Jetson),
    # which publishes execution_mode "real" on this same topic.
    detail: str | None = None
    flight_mode: str | None = None


class MultiStepFlightTestStatusResponse(BaseModel):
    """Snapshot of /flight_test/multi_step/status (multi_step_test_node)
    -- a DELIBERATELY SEPARATE type from FlightTestStatusResponse above,
    same convention as every other status-response pair in this file:
    hover's state machine only ever reaches "aborted" as a terminal
    failure state, while the multi-step scenarios have a distinct
    "failed" state (the scenario's own execution went wrong, e.g. an
    unrecognized action) that "aborted" (operator hit ABORT) must never
    be conflated with -- see onboard-autonomy's multi_step_test_node."""

    scenario_id: str | None = None
    state: str | None = None
    phase: str | None = None
    current_step_index: int | None = None
    current_step_action: str | None = None
    total_steps: int | None = None
    current_position: list[float] | None = None
    armed: bool | None = None
    execution_mode: str | None = None


class MissionStartRequest(BaseModel):
    mission: str


class RadioCommandResponse(BaseModel):
    """A START/ABORT the Jetson ACKNOWLEDGED AND ACCEPTED over the radio.
    Only ever returned with status "accepted" -- no ACK and rejections are
    HTTP errors (504 / 409), never a 200."""

    status: str
    command: str
    mission: str | None = None
    nonce: int
    attempts: int


class RadioLastCommand(BaseModel):
    command: str
    mission_code: int
    nonce: int
    attempts: int
    acked: bool
    result: int | None = None
    reason: str | None = None
    time: str = ""


class ComponentStatus(BaseModel):
    """One drone part on the Components panel. status: "ok" (its data keeps
    arriving), "lost" (it stopped), "missing" (never heard)."""

    key: str
    name: str
    status: str
    age_s: float | None = None
    detail: str | None = None


class ComponentsResponse(BaseModel):
    """available=False: not on the radio telemetry path (rosbridge/sim),
    where the Components panel has nothing to report."""

    available: bool = False
    components: list[ComponentStatus] = []


class RadioStatusResponse(BaseModel):
    enabled: bool
    port: str | None = None
    baud: int | None = None
    port_open: bool
    port_error: str | None = None
    # Jetson heartbeats received over the radio within the last few seconds.
    jetson_link_up: bool
    jetson_heartbeat_age_s: float | None = None
    # Mission state carried in the Jetson's radio heartbeat (works without Wi-Fi).
    jetson_mission_state: str | None = None
    # Which mission that state belongs to ("hover", "motor_test"; None = unknown).
    jetson_mission: str | None = None
    # Radio diagnostics: messages received per MAVLink type since start, and
    # frames that failed to parse/checksum (garbled on the radio).
    rx_counts: dict[str, int] = {}
    rx_bad_bytes: int = 0
    last_command: RadioLastCommand | None = None


class SimulationCommandResponse(BaseModel):
    """Deliberately a DIFFERENT type from CommandResponse -- see
    SimulationStatusResponse's own docstring for why this repo never
    reuses a real-telemetry/real-command response type for simulation
    data, even where the shape would otherwise match."""

    status: str
    command: str


class SimulationStatusResponse(BaseModel):
    """The simulation's own status -- a DELIBERATELY DIFFERENT Pydantic
    type from TelemetryResponse, not just a relabeled copy, so a
    simulation response can never be structurally confused with real
    Pixhawk telemetry even by a caller that forgot to check `source`.
    See CHECKPOINT/CURRENT_STATE.md and
    onboard-autonomy/nidar_autonomy/simulation_node.py, the only thing
    that ever produces the data behind this response.

    `status` is the simulation's own lifecycle ("idle"/"running"/
    "completed"/"failed") -- distinct from `mission_state`, which is the
    simulated MISSION's lifecycle (idle/entering/searching/exiting/
    complete/aborted, same vocabulary as the real system's
    /mission/state, but from this simulator's own, separate state
    machine instance)."""

    source: str = "simulation"
    status: str = "idle"
    mission_state: str = "idle"
    step: int = 0
    elapsed_sim_seconds: float = 0.0
    pose: PositionResponse | None = None
    autonomy: AutonomyStateResponse = AutonomyStateResponse()
    sensors: SensorsResponse = SensorsResponse()
    mapping: MappingStatusResponse = MappingStatusResponse()
    navigation: NavigationResponse = NavigationResponse()
    # Two different metrics -- see
    # onboard-autonomy/nidar_autonomy/mission_simulator.py's
    # SimulationSnapshot docstring for why they diverge sharply and both
    # matter: map_known_pct is "how much of the arena has been
    # discovered" (grows progressively -- render this as the headline
    # exploration-progress indicator); coverage_search_pct is "of what's
    # currently known, how much has the camera actually searched"
    # (saturates near 100% quickly -- a different, narrower question).
    map_known_pct: float = 0.0
    coverage_search_pct: float = 0.0
    error: str | None = None


class ParamWriteRequest(BaseModel):
    """Bench Setup page only (GCS_SETUP_ENABLED)."""

    name: str
    value: float


class ParamResponse(BaseModel):
    name: str
    value: float
    attempts: int


class LidarResponse(BaseModel):
    """Live LiDAR scan reduced to sectors (radio: OBSTACLE_DISTANCE from the
    Jetson). Angles clockwise from the drone's nose; distances_cm[i] is the
    nearest return at angle_offset_deg + i*increment_deg, None = no return."""

    available: bool = False
    age_s: float | None = None
    angle_offset_deg: float = 0.0
    increment_deg: float = 5.0
    min_cm: int | None = None
    max_cm: int | None = None
    distances_cm: list[int | None] = []

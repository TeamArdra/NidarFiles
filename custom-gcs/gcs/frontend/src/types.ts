// Mirrors gcs/backend/app/schemas.py exactly. Keep in sync by hand --
// there is no shared codegen between the Python and TypeScript sides
// (see custom-gcs/CLAUDE.md: "Docs and interfaces before code" -- this
// file is the frontend half of that contract).

// "connected": ROS enabled and rosbridge reachable right now.
// "disabled": the backend was started with GCS_ROS_ENABLED=false --
//   ROS-optional local-development mode (no Jetson, no ROS, no rosbridge
//   available at all, e.g. a plain Windows laptop) -- and never attempts
//   any ROS/rosbridge connectivity. Mission/scenario metadata still
//   works; START/ABORT/mission-start/simulation commands 503 instead of
//   pretending to succeed.
// "unavailable": ROS is enabled but rosbridge is not currently reachable
//   (Jetson off, rosbridge_server not running, wrong host/port, network
//   down, ...). Deliberately distinct from "disabled" so the operator can
//   tell "we chose not to connect" apart from "we tried and failed."
// See gcs/backend/app/schemas.py's HealthResponse docstring.
export type RosStatus = "connected" | "disabled" | "unavailable";

export interface HealthResponse {
  connected: boolean;
  ros_status: RosStatus;
  // "radio": telemetry over the MicroLR900 radio (no Wi-Fi link);
  // "rosbridge": the Wi-Fi/rosbridge path (development against sim/).
  telemetry_source?: "radio" | "rosbridge";
  // True only when the backend was started for bench setup (start_gcs.ps1 -Setup).
  setup_enabled?: boolean;
  rosbridge_host: string;
  rosbridge_port: number;
}

export interface BatteryResponse {
  voltage: number | null;
  current: number | null;
  percentage: number | null;
}

export interface PositionResponse {
  x: number;
  y: number;
  z: number;
}

export interface PoseResponse {
  position: PositionResponse | null;
}

export interface VelocityResponse {
  x: number;
  y: number;
  z: number;
}

// Orientation quaternion straight from /mavros/imu/data -- left as a
// quaternion rather than converted to Euler angles, so the frontend
// doesn't silently pick a convention the backend didn't ask for either.
export interface AttitudeResponse {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface GpsResponse {
  fix_status: number | null;
  satellites_visible: number | null;
  latitude: number | null;
  longitude: number | null;
  altitude: number | null;
}

export interface StatusTextResponse {
  severity: number;
  text: string;
}

// `connected` here is the FCU<->MAVROS link (distinct from
// TelemetryResponse.connected, which is the GCS<->rosbridge link) --
// both can fail independently and the operator needs to tell them apart.
export interface FcuStateResponse {
  connected: boolean | null;
  armed: boolean | null;
  guided: boolean | null;
  mode: string | null;
  system_status: number | null;
}

// Structured "Active Thinking" panel state -- fixed vocabulary only, never
// free-form/LLM-generated text. See
// CHECKPOINT/docs/gcs_telemetry_contract.md.
export interface AutonomyStateResponse {
  state: string | null;
  objective: string | null;
  target: [number, number] | null;
  next_action: string | null;
}

export interface SensorsResponse {
  slam: string | null;
  lidar: string | null;
  rangefinder: string | null;
  camera: string | null;
}

// Lightweight mapping *summary* -- the full occupancy grid is fetched
// separately via getMap(), not duplicated here.
export interface MappingStatusResponse {
  available: boolean;
  resolution_m: number | null;
  width_cells: number | null;
  height_cells: number | null;
  origin_x: number | null;
  origin_y: number | null;
  coverage_cell_size_m: number | null;
  explored_pct: number | null;
}

export interface NavigationResponse {
  target: [number, number] | null;
  frontier_count: number | null;
  candidate_count: number | null;
  blacklisted_count: number | null;
  geofence_breached: boolean | null;
}

export interface TelemetryResponse {
  connected: boolean;
  mission_state: string | null;
  fcu: FcuStateResponse;
  battery: BatteryResponse;
  pose: PoseResponse;
  velocity: VelocityResponse | null;
  attitude: AttitudeResponse | null;
  gps: GpsResponse | null;
  statustext: StatusTextResponse[];
  heartbeat_age_s: number | null;
  autonomy: AutonomyStateResponse;
  sensors: SensorsResponse;
  mapping: MappingStatusResponse;
  navigation: NavigationResponse;
}

export interface MapResponse {
  resolution: number | null;
  width: number | null;
  height: number | null;
  data: number[] | null;
  // Radio SLAM map only (start_jetson.sh, unless --no-map): map-frame origin of cell
  // (0,0), the LiDAR/drone pose from SLAM, and how old the map is.
  origin_x?: number | null;
  origin_y?: number | null;
  robot?: { x: number; y: number; yaw_deg: number } | null;
  age_s?: number | null;
  // The map as the organiser's 1 m grid (radio ARENA_GRID), if the Jetson
  // has aligned one yet -- see ArenaGrid.
  arena?: ArenaGrid | null;
}

// Edge states: 0 unknown, 1 open, 2 wall.
export type ArenaEdge = 0 | 1 | 2;

// The SLAM map as the organiser's 1 m arena grid (backend ArenaGridResponse).
// Arena frame: u = x cos(theta) + y sin(theta), v = -x sin(theta) + y cos(theta)
// in map-frame metres; cell (col, row) spans u from offset_u + entry_u + col,
// v from offset_v + entry_v + row, 1 m each. Cell (0, 0) = take-off cell.
// v_edges[r][k]: west edge of column col0 + k in row row0 + r (k = cols: east
// edge of the last column); h_edges[k][c]: south edge of row row0 + k in
// column col0 + c (k = rows: north edge of the last row).
export interface ArenaGrid {
  theta_deg: number;
  offset_u: number;
  offset_v: number;
  entry_u: number;
  entry_v: number;
  col0: number;
  row0: number;
  cols: number;
  rows: number;
  v_edges: ArenaEdge[][];
  h_edges: ArenaEdge[][];
  age_s?: number | null;
}

// Same shape as MapResponse -- the coverage grid is also a
// nav_msgs/OccupancyGrid, just with different cell semantics (-1=unknown,
// 0=free-not-yet-searched, 100=searched).
export interface CoverageResponse {
  resolution: number | null;
  width: number | null;
  height: number | null;
  data: number[] | null;
}

export interface PathPointResponse {
  x: number;
  y: number;
}

export interface PathResponse {
  points: PathPointResponse[];
}

export interface FrontierPointResponse {
  x: number;
  y: number;
}

export interface FrontiersResponse {
  points: FrontierPointResponse[];
}

export interface SurvivorResponse {
  survivor_id: number;
  x: number;
  y: number;
  confidence: number;
}

// -- Perception (dev-only pretrained person-detector, image-space, raw
// detections) -- deliberately NOT SurvivorResponse: a detection here is
// unconfirmed and in image (pixel) coordinates, not a localized,
// world-x/y survivor tag. See custom-gcs/CLAUDE.md and
// gcs/backend/app/schemas.py's perception models.

export interface BBox {
  x_min: number | null;
  y_min: number | null;
  x_max: number | null;
  y_max: number | null;
}

export interface Detection {
  detection_id: string | null;
  class_name: string | null;
  confidence: number | null;
  bbox: BBox;
  center_x: number | null;
  center_y: number | null;
  track_id: string | null;
  source: string | null;
  model_name: string | null;
}

export interface PerceptionDetectionsResponse {
  frame_width: number | null;
  frame_height: number | null;
  timestamp: number | null;
  detections: Detection[];
}

export interface PerceptionStatusResponse {
  camera_connected: boolean | null;
  detector_enabled: boolean | null;
  detector_ready: boolean | null;
  detector_backend: string | null;
  model_name: string | null;
  person_count: number | null;
  fps: number | null;
  frame_width: number | null;
  frame_height: number | null;
  last_detection_age_s: number | null;
}

// Video bytes bypass the backend entirely (see docs/DECISIONS.md D-6) --
// this endpoint only tells the frontend where to point an <img> tag, it
// does not itself carry any video data.
export interface CameraStatusResponse {
  connected: boolean | null;
  stream_url: string | null;
  frame_width: number | null;
  frame_height: number | null;
  fps: number | null;
}

// START/ABORT go over the MicroLR900 command radio (backend
// app/radio_link.py). A 200 means the Jetson ACKed AND accepted the
// command; no-ACK (504) and rejection (409) arrive as errors. The operator
// command surface is exactly START (for a selected mission) and ABORT --
// see custom-gcs/CLAUDE.md Important Constraints #1.
export interface RadioCommandResponse {
  status: "accepted";
  command: "start" | "abort";
  mission: string | null;
  nonce: number;
  attempts: number;
}

export interface RadioLastCommand {
  command: string;
  mission_code: number;
  nonce: number;
  attempts: number;
  acked: boolean;
  result: number | null;
  reason: string | null;
  time: string;
}

export interface RadioStatusResponse {
  enabled: boolean;
  port: string | null;
  baud: number | null;
  port_open: boolean;
  port_error: string | null;
  jetson_link_up: boolean;
  jetson_heartbeat_age_s: number | null;
  jetson_mission_state: string | null;
  // Which mission that state belongs to ("hover", "motor_test"); null = unknown.
  jetson_mission?: string | null;
  last_command: RadioLastCommand | null;
}

// The Mission dropdown -- mirrors gcs/backend/app/missions.py. Choosing a
// mission is configuration, not an operator command.
export interface Mission {
  id: string;
  name: string;
  description: string;
}

// /flight_test/status -- published by the real hover mission on the
// Jetson (execution_mode "real"). Arrives over the radio.
export interface FlightTestStatusResponse {
  scenario: string | null;
  state: string | null;
  target_altitude_m: number | null;
  current_altitude_m: number | null;
  current_position: [number, number, number] | null;
  duration_s: number | null;
  elapsed_hover_s: number | null;
  armed: boolean | null;
  execution_mode: string | null;
  detail: string | null;
  flight_mode: string | null;
}

// Status readback for the two multi-step flight-test scenarios
// (multi_step_test_node) -- a DELIBERATELY SEPARATE type from
// FlightTestStatusResponse above, same convention as every other
// status-response pair in this codebase. Note the distinct "failed"
// state (the scenario's own execution went wrong) vs. "aborted" (the
// operator hit ABORT) -- hover's simpler state machine has no "failed".
export interface MultiStepFlightTestStatusResponse {
  scenario_id: string | null;
  state: "idle" | "arming" | "executing" | "landing" | "complete" | "aborted" | "failed" | null;
  phase: "idle" | "taking_off" | "step" | "landing" | null;
  current_step_index: number | null;
  current_step_action: string | null;
  total_steps: number | null;
  current_position: [number, number, number] | null;
  armed: boolean | null;
  execution_mode: string | null;
}

// -- Simulation ("RUN SIMULATION") -- completely separate from the real
// mission command/telemetry types above. See
// CHECKPOINT/CURRENT_STATE.md and
// onboard-autonomy/nidar_autonomy/simulation_node.py.

export interface SimulationCommandResponse {
  status: string;
  command: string;
}

// Deliberately NOT TelemetryResponse -- see gcs/backend/app/schemas.py's
// SimulationStatusResponse docstring for why this stays a distinct type
// even though several nested shapes match.
export interface SimulationStatusResponse {
  source: "simulation";
  status: "idle" | "running" | "completed" | "failed";
  mission_state: string;
  step: number;
  elapsed_sim_seconds: number;
  pose: PositionResponse | null;
  autonomy: AutonomyStateResponse;
  sensors: SensorsResponse;
  mapping: MappingStatusResponse;
  navigation: NavigationResponse;
  map_known_pct: number;
  coverage_search_pct: number;
  error: string | null;
}

// The simulation control surface is exactly these two -- never
// "start"/"abort" (that vocabulary stays exclusive to the real command path above).
export type SimulationCommand = "run" | "reset";

// Bench Setup page: a Pixhawk parameter as read back from the FCU.
export interface ParamResponse {
  name: string;
  value: number;
  attempts: number;
}

// Live LiDAR sectors (radio OBSTACLE_DISTANCE from the Jetson). Angles
// clockwise from the nose; null = no return in that sector.
export interface LidarResponse {
  available: boolean;
  age_s: number | null;
  angle_offset_deg: number;
  increment_deg: number;
  min_cm: number | null;
  max_cm: number | null;
  distances_cm: (number | null)[];
}

// /api/components: every drone part the GCS hears from over the radio.
// "ok" = its data keeps arriving, "lost" = it stopped, "missing" = never heard.
export type ComponentState = "ok" | "lost" | "missing";

export interface ComponentStatus {
  key: string;
  name: string;
  status: ComponentState;
  age_s: number | null;
  detail: string | null;
}

export interface ComponentsResponse {
  available: boolean;
  components: ComponentStatus[];
}

"""ArduCopter (4.6.x) vehicle control through MAVROS -- the minimal set of
operations a GUIDED-mode mission needs: mode changes, arm/disarm, takeoff,
and a fresh view of state/position/battery -- plus ArduCopter's motor test
(MAV_CMD_DO_MOTOR_TEST, props-off bench check; missions/motor_test).

ArduCopter, NOT PX4: there is no OFFBOARD mode and no setpoint stream is
needed to stay in GUIDED. After a GUIDED takeoff ArduCopter holds its
position on its own; LAND mode descends and the FCU disarms itself after
touchdown. That is why this layer has no setpoint publisher at all.

Arm/disarm are NOT reimplemented here: they go through the existing,
bench-verified flight_command.FlightCommandClient (still the only code in
this repo that calls the arming service -- Hard Safety Rule 1). Its
blocking calls run on worker threads so the node's executor keeps
processing the /mavros/state updates they wait on (same pattern as
mission_state_node.py, with FlightCommandClient(already_spinning=True)).

Every operation is non-blocking (safe to call from a timer callback) and reports through a `done(ok, detail)`
callback; callers must confirm the effect from snapshot() (e.g. that the
mode really changed), not from the service reply alone.

EKF origin: indoors there is no GPS, so the EKF never gets an origin by
itself, and without one ArduCopter refuses a GUIDED takeoff (it cannot
turn the takeoff into a target position) even though a local position is
published. ensure_ekf_origin() sets a fixed origin through MAVROS
(SET_GPS_GLOBAL_ORIGIN) once per FCU connection, then keeps asking the
FCU to report it (GPS_GLOBAL_ORIGIN) so snapshot().ekf_origin_age_s shows
it is really set. Seen on the bench 2026-10-08: no origin -> TAKEOFF
refused after ARM.

HARDWARE VERIFICATION REQUIRED (not testable without the vehicle):
  - service names /mavros/set_mode, /mavros/cmd/takeoff,
    /mavros/set_message_interval exist on the Jetson's MAVROS build;
  - GUIDED takeoff via /mavros/cmd/takeoff climbs to `altitude` metres
    above the arming point on this vehicle;
  - /mavros/local_position/pose is published once the EKF has a local
    position estimate from the indoor position source.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, CommandTOL, MessageInterval, SetMode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

from .arming_guard import ArmRejected
from .flight_command import FlightCommandClient
from .vehicle_snapshot import VehicleSnapshot

GUIDED = "GUIDED"
LAND = "LAND"

SET_MODE_SERVICE = "/mavros/set_mode"
TAKEOFF_SERVICE = "/mavros/cmd/takeoff"
MESSAGE_INTERVAL_SERVICE = "/mavros/set_message_interval"
COMMAND_SERVICE = "/mavros/cmd/command"
ORIGIN_TOPIC = "/mavros/global_position/gp_origin"
SET_ORIGIN_TOPIC = "/mavros/global_position/set_gp_origin"

MAV_CMD_REQUEST_MESSAGE = 512
MSG_ID_GPS_GLOBAL_ORIGIN = 49
ORIGIN_RETRY_S = 2.0
# The origin counts as confirmed while the FCU reported it this recently.
ORIGIN_FRESH_S = 3.0

MAV_CMD_DO_MOTOR_TEST = 209
MOTOR_TEST_THROTTLE_PERCENT = 0
# Motor numbering by ArduPilot's test sequence: 1 = A (front-right on a
# quad X), then clockwise B, C, D -- the letters in ArduPilot's frame
# diagrams and Mission Planner's motor test page.
MOTOR_TEST_ORDER_SEQUENCE = 1
MAV_RESULT_ACCEPTED = 0
STATE_TOPIC = "/mavros/state"
LOCAL_POSITION_TOPIC = "/mavros/local_position/pose"
BATTERY_TOPIC = "/mavros/battery"

# MAVLink message id -> rate (Hz) requested from the FCU at startup.
# ArduPilot's stream rates reset when MAVROS/the Pixhawk restarts (see
# custom-gcs/NIDAR_AirMouse_Daily_Startup_SOP.md Phase 5), and the hover
# mission depends on fresh local position.
STREAM_RATES_HZ = {
    32: 20.0,  # LOCAL_POSITION_NED -> /mavros/local_position/pose
    # ATTITUDE drives /mavros/imu/data and RAW_IMU gives it its acceleration:
    # Cartographer uses that IMU (start_jetson.sh --map), so a START must not
    # drop it below the 50 Hz start_jetson.sh asked for.
    30: 50.0,  # ATTITUDE
    27: 50.0,  # RAW_IMU
    1: 2.0,  # SYS_STATUS
    147: 1.0,  # BATTERY_STATUS -> /mavros/battery
    49: 1.0,  # GPS_GLOBAL_ORIGIN -> /mavros/global_position/gp_origin (once set)
}

Done = Callable[[bool, str], None]


class ArduCopterVehicle:
    def __init__(self, node: Node, flight: FlightCommandClient) -> None:
        self._node = node
        self._log = node.get_logger()
        self._flight = flight
        self._flight_lock = threading.Lock()

        self._state: Optional[State] = None
        self._state_at: Optional[float] = None
        self._position = None
        self._position_at: Optional[float] = None
        self._battery_v: Optional[float] = None
        self._battery_at: Optional[float] = None
        self._origin_at: Optional[float] = None
        self._origin = None  # (lat, lon, alt) as reported by the FCU
        self._origin_sent_this_link = False
        self._origin_last_try: Optional[float] = None

        # BEST_EFFORT matches both reliable and best-effort MAVROS publishers.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        node.create_subscription(State, STATE_TOPIC, self._on_state, qos)
        node.create_subscription(PoseStamped, LOCAL_POSITION_TOPIC, self._on_pose, qos)
        node.create_subscription(BatteryState, BATTERY_TOPIC, self._on_battery, qos)
        node.create_subscription(GeoPointStamped, ORIGIN_TOPIC, self._on_origin, qos)
        self._set_origin_pub = node.create_publisher(GeoPointStamped, SET_ORIGIN_TOPIC, 10)

        self._set_mode_client = node.create_client(SetMode, SET_MODE_SERVICE)
        self._takeoff_client = node.create_client(CommandTOL, TAKEOFF_SERVICE)
        self._interval_client = node.create_client(MessageInterval, MESSAGE_INTERVAL_SERVICE)
        self._command_client = node.create_client(CommandLong, COMMAND_SERVICE)

    # -- state ----------------------------------------------------------------

    def _on_state(self, msg: State) -> None:
        self._state = msg
        self._state_at = time.monotonic()

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._position = (p.x, p.y, p.z)
        self._position_at = time.monotonic()

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_v = float(msg.voltage)
        self._battery_at = time.monotonic()

    def _on_origin(self, msg: GeoPointStamped) -> None:
        p = msg.position
        if self._origin_at is None:
            self._log.warning(
                f"[ardupilot_vehicle] EKF origin confirmed by FCU: lat {p.latitude:.6f} "
                f"lon {p.longitude:.6f} alt {p.altitude:.1f}"
            )
        self._origin = (p.latitude, p.longitude, p.altitude)
        self._origin_at = time.monotonic()

    def snapshot(self) -> VehicleSnapshot:
        now = time.monotonic()
        age = lambda t: None if t is None else now - t  # noqa: E731
        state = self._state
        return VehicleSnapshot(
            fcu_connected=bool(state and state.connected),
            state_age_s=age(self._state_at),
            armed=None if state is None else bool(state.armed),
            mode=None if state is None else state.mode,
            position=self._position,
            position_age_s=age(self._position_at),
            battery_voltage_v=self._battery_v,
            battery_age_s=age(self._battery_at),
            ekf_origin_age_s=age(self._origin_at),
        )

    # -- commands -------------------------------------------------------------

    def set_mode(self, mode: str, done: Done) -> None:
        if not self._set_mode_client.service_is_ready():
            done(False, f"{SET_MODE_SERVICE} not available")
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self._log.warning(f"[ardupilot_vehicle] set_mode {mode}")
        future = self._set_mode_client.call_async(request)
        future.add_done_callback(
            lambda f: self._finish(f, done, lambda r: (r.mode_sent, f"mode_sent={r.mode_sent}"))
        )

    def takeoff(self, altitude_m: float, done: Done) -> None:
        if not self._takeoff_client.service_is_ready():
            done(False, f"{TAKEOFF_SERVICE} not available")
            return
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = float(altitude_m)
        self._log.warning(f"[ardupilot_vehicle] TAKEOFF to {altitude_m:.2f} m")
        future = self._takeoff_client.call_async(request)
        future.add_done_callback(
            lambda f: self._finish(
                f, done, lambda r: (r.success, f"success={r.success} result={r.result}")
            )
        )

    def motor_test(self, motor: int, throttle_pct: float, timeout_s: float, done: Done) -> None:
        """Spin one motor (1-based, test-sequence order) at `throttle_pct`
        for `timeout_s` seconds via ArduCopter's motor test. The FCU stops
        the motor by itself when the timeout ends, so a lost Jetson cannot
        leave it spinning. throttle 0 with timeout 0 stops a running test
        at once. ArduCopter refuses while armed, with the safety switch not
        pressed, or with RC not calibrated -- the reason is in its
        STATUSTEXT."""
        if not self._command_client.service_is_ready():
            done(False, f"{COMMAND_SERVICE} not available")
            return
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_MOTOR_TEST
        request.confirmation = 0
        request.param1 = float(motor)
        request.param2 = float(MOTOR_TEST_THROTTLE_PERCENT)
        request.param3 = float(throttle_pct)
        request.param4 = float(timeout_s)
        request.param5 = 0.0  # motor count 0 = this motor only
        request.param6 = float(MOTOR_TEST_ORDER_SEQUENCE)
        request.param7 = 0.0
        self._log.warning(
            f"[ardupilot_vehicle] MOTOR TEST motor {motor} at {throttle_pct:.0f}% for {timeout_s:.1f} s"
        )
        future = self._command_client.call_async(request)
        future.add_done_callback(
            lambda f: self._finish(
                f,
                done,
                lambda r: (
                    r.success and r.result == MAV_RESULT_ACCEPTED,
                    f"success={r.success} result={r.result}",
                ),
            )
        )

    def arm(self, done: Done) -> None:
        threading.Thread(target=self._arming, args=(True, done), daemon=True).start()

    def disarm(self, done: Done) -> None:
        threading.Thread(target=self._arming, args=(False, done), daemon=True).start()

    def _arming(self, arm: bool, done: Done) -> None:
        with self._flight_lock:
            try:
                result = self._flight.arm() if arm else self._flight.disarm()
            except ArmRejected as exc:
                done(False, str(exc))
                return
        done(result.success and result.state_confirmed, result.message)

    def ensure_ekf_origin(self, lat: float, lon: float, alt: float) -> None:
        """Call periodically. While the FCU hasn't recently reported an
        origin: send SET_GPS_GLOBAL_ORIGIN (once per FCU connection, so an
        existing origin is never moved in flight) and ask the FCU to report
        it. Call reset_link() when the FCU (re)connects."""
        now = time.monotonic()
        if self._origin_at is not None and now - self._origin_at < ORIGIN_FRESH_S:
            return
        if self._origin_last_try is not None and now - self._origin_last_try < ORIGIN_RETRY_S:
            return
        self._origin_last_try = now
        if not self._origin_sent_this_link:
            msg = GeoPointStamped()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.position.latitude = float(lat)
            msg.position.longitude = float(lon)
            msg.position.altitude = float(alt)
            self._set_origin_pub.publish(msg)
            self._origin_sent_this_link = True
            self._log.warning(
                f"[ardupilot_vehicle] setting EKF origin: lat {lat:.6f} lon {lon:.6f} alt {alt:.1f}"
            )
        self._request_message(MSG_ID_GPS_GLOBAL_ORIGIN)

    def reset_link(self) -> None:
        """The FCU (re)connected -- e.g. after a reboot it has no origin."""
        self._origin_sent_this_link = False
        self._origin_last_try = None
        self._origin_at = None

    def _request_message(self, message_id: int) -> None:
        if not self._command_client.service_is_ready():
            return
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_REQUEST_MESSAGE
        request.confirmation = 0
        request.param1 = float(message_id)
        self._command_client.call_async(request)

    def request_streams(self) -> None:
        """Best effort: ask the FCU for the message rates this layer needs.
        Logged, never fatal -- the mission's own freshness checks are what
        actually gate flight."""
        if not self._interval_client.service_is_ready():
            self._log.error(
                f"[ardupilot_vehicle] {MESSAGE_INTERVAL_SERVICE} not available -- "
                "stream rates not requested (restore them manually per the SOP)"
            )
            return
        for message_id, rate in STREAM_RATES_HZ.items():
            request = MessageInterval.Request()
            request.message_id = message_id
            request.message_rate = rate
            self._interval_client.call_async(request).add_done_callback(
                lambda f, m=message_id, r=rate: self._log.info(
                    f"[ardupilot_vehicle] stream {m} @ {r} Hz: "
                    f"{'ok' if f.result() and f.result().success else 'NOT confirmed'}"
                )
            )

    @staticmethod
    def _finish(future, done: Done, interpret) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- service/transport failure
            done(False, f"service call failed: {exc!r}")
            return
        if response is None:
            done(False, "no service response")
            return
        ok, detail = interpret(response)
        done(bool(ok), detail)

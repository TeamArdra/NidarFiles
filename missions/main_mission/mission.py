#!/usr/bin/env python3
"""NIDAR Main Mission -- ROS 2 node, runs on the Jetson (real drone) or on
the sim laptop (ArduPilot SITL + Gazebo, start_sim.sh).

    radio START (main_mission / main_mission_sim) -> radio_command_node
      -> /gcs/mission_select + /gcs/command -> command_node (validation)
      -> /nidar_autonomy/validated_command -> THIS NODE
      -> ArduCopterVehicle -> MAVROS -> Pixhawk 6X / ArduCopter SITL

    python3 missions/main_mission/mission.py          Main Mission (REAL DRONE)
    python3 missions/main_mission/mission.py --sim    Main Mission (Sim)

The two are the same code and the same decisions (main_mission_logic.py);
only CONFIG/CONFIG_SIM and the mission id differ. A guard keeps them
apart: the sim mission refuses START unless NIDAR_SIM=1 (set only by
start_sim.sh), the real one refuses START when NIDAR_SIM=1 -- so selecting
the wrong one in the GCS can never fly the drone with sim settings.

Inputs besides the FCU: /map (Cartographer's occupancy grid) and TF
map -> laser (the LiDAR's SLAM pose; NIDAR_BODY_FRAME to change). Like the
hover node, every callback is guarded: an unexpected exception lands the
vehicle instead of killing the node.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformException, TransformListener  # noqa: E402

from explore_logic import ExploreConfig, FollowConfig  # noqa: E402
from main_mission_logic import MainConfig, MainMission, Perception  # noqa: E402
from nidar_autonomy.ardupilot_vehicle import ArduCopterVehicle  # noqa: E402
from nidar_autonomy.flight_command import FlightCommandClient  # noqa: E402
from nidar_autonomy.geometry import yaw_from_quaternion  # noqa: E402
from nidar_autonomy.mission_selection import MissionSelection  # noqa: E402
from nidar_autonomy.ros_conversions import occupancy_grid_msg_to_dict  # noqa: E402
from nidar_autonomy.topics import (  # noqa: E402
    MAIN_MISSION_SIM_STATUS_TOPIC,
    MAIN_MISSION_STATUS_TOPIC,
    MAP_TOPIC,
    MISSION_SELECT_TOPIC,
    VALIDATED_COMMAND_TOPIC,
)

# ---------------------------------------------------------------------------
# Flight parameters. Edit here.
# TODO(hardware): review every REAL value in the test space before the first
# flight (README section 7.5): altitude inside the rangefinder's range and
# well below the net; inflation >= the drone's real radius with guards;
# battery limits for the flight pack. Start with a short explore budget.
# ---------------------------------------------------------------------------
CONFIG = MainConfig(
    takeoff_altitude_m=1.0,
    cruise_speed_mps=0.3,
    explore_budget_s=600.0,  # 10 min exploring, so return + land fit the 15 min bonus
    geofence_m=20.0,
    min_battery_voltage_v=14.7,  # 4S at rest (= BATT_ARM_VOLT)
    low_battery_v=14.5,  # -> return to the entry
    critical_battery_v=14.0,  # -> land where it is (BATT_CRT_VOLT also lands)
    explore=ExploreConfig(inflation_m=0.30),  # 1 m corridors: leaves ~0.3 m of path width
    follow=FollowConfig(lookahead_m=0.5),
)

# ArduPilot SITL iris in the Gazebo maze (2 m corridors, iris ~0.55 m wide).
CONFIG_SIM = MainConfig(
    takeoff_altitude_m=1.0,
    cruise_speed_mps=0.5,
    explore_budget_s=600.0,
    geofence_m=20.0,
    min_battery_voltage_v=14.7,  # SITL battery is set to 4S in sim/params/nidar_sitl.parm
    low_battery_v=14.5,
    critical_battery_v=14.0,
    explore=ExploreConfig(inflation_m=0.45),
    follow=FollowConfig(lookahead_m=0.6),
    # Gazebo in WSL can run well below real time (software rendering of the
    # LiDAR): the mission clock is sim time (_mission_now below), but message ages
    # are measured in wall time, so messages arrive "late" -- looser limits.
    state_stale_s=10.0,
    position_stale_s=3.0,
    map_stale_s=15.0,
)

# Same EKF origin as the hover mission (no GPS indoors).
EKF_ORIGIN = (12.9165, 79.1325, 216.0)
BODY_FRAME = os.environ.get("NIDAR_BODY_FRAME", "laser")
MAP_FRAME = "map"

_CONFLICTING_NODES = ("mission_state_node",)
_LAST_RESORT = " -- TAKE OVER ON THE RC TRANSMITTER"


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


class MainMissionNode(Node):
    def __init__(self, sim: bool) -> None:
        self._sim = sim
        self._mission_id = "main_mission_sim" if sim else "main_mission"
        super().__init__(f"{self._mission_id}_mission")
        self._config = CONFIG_SIM if sim else CONFIG
        self._lock = threading.RLock()
        self._mission = MainMission(self._config, self._mission_id)
        # The sim's /mavros/state can arrive seconds apart (slower than real
        # time): give ARM/DISARM longer to show up there. The drone keeps 2 s.
        flight = (
            FlightCommandClient(self, already_spinning=True, state_confirm_timeout_sec=10.0)
            if sim else FlightCommandClient(self, already_spinning=True)
        )
        self._vehicle = ArduCopterVehicle(self, flight)
        # one selection per START (TeamArdra failsafe, mission_selection.py)
        self._selection = MissionSelection()
        self._streams_requested = False
        self._last_logged = (self._mission.state, self._mission.detail)
        self._map = None  # (dict, monotonic receive time)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._guarded("map", self._on_map), map_qos)
        self.create_subscription(String, MISSION_SELECT_TOPIC, self._guarded("selection", self._on_select), 10)
        self.create_subscription(String, VALIDATED_COMMAND_TOPIC, self._guarded("command", self._on_command), 10)
        status_topic = MAIN_MISSION_SIM_STATUS_TOPIC if sim else MAIN_MISSION_STATUS_TOPIC
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(0.1, self._guarded("tick", self._tick))
        self.create_timer(0.5, self._guarded("status", self._publish_status))

        kind = "SIM (ArduPilot SITL)" if sim else "REAL FLIGHT"
        self.get_logger().warning(
            f"[{_now()}] {self._mission_id} ready ({kind}): takeoff {self._config.takeoff_altitude_m} m, "
            f"explore up to {self._config.explore_budget_s:.0f} s; waiting for radio START"
        )

    # -- inputs -------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = (occupancy_grid_msg_to_dict(msg), time.monotonic())

    def _mission_now(self) -> float:
        """The mission's `now`: wall (monotonic) on the drone; sim seconds in
        the simulator, so timeouts and budgets keep their meaning when
        Gazebo runs slower than real time."""
        if self._sim:
            return self.get_clock().now().nanoseconds / 1e9
        return time.monotonic()

    def _perception(self) -> Perception:
        grid, map_age = (None, None) if self._map is None else (self._map[0], time.monotonic() - self._map[1])
        pose = age = None
        try:
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, BODY_FRAME, Time())
        except TransformException:
            tf = None
        if tf is not None:
            t, q = tf.transform.translation, tf.transform.rotation
            pose = (t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            stamp = Time.from_msg(tf.header.stamp)
            age = max(0.0, (self.get_clock().now() - stamp).nanoseconds / 1e9)
        return Perception(grid, map_age, pose, age)

    def _on_select(self, msg: String) -> None:
        try:
            selection = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(f"ignoring malformed mission selection {msg.data!r}")
            return
        if isinstance(selection, dict):
            self._selection.update(selection.get("mission_id"), time.monotonic())

    def _on_command(self, msg: String) -> None:
        now = self._mission_now()
        wall = time.monotonic()  # the mission selection is timed in wall time
        snap = self._vehicle.snapshot()
        with self._lock:
            if msg.data == "abort":
                self.get_logger().error(f"[{_now()}] ABORT received (state {self._mission.state})")
                self._run(self._mission.abort(now, snap))
                return
            if msg.data != "start":
                return
            for_us, problem = self._selection.take_for(self._mission_id, wall)
            if not for_us:
                return  # another mission's START: not ours, not a failure
            problem = problem or self._start_problem()
            if problem:
                self.get_logger().error(f"[{_now()}] START refused: {problem}")
                self._mission.reject_start(problem)
                self._log_change()
                return
            self.get_logger().warning(f"[{_now()}] START received for {self._mission_id}")
            self._run(self._mission.start(now, snap, self._perception()))

    def _start_problem(self):
        """Checks beyond the mission selection (already checked by take_for)."""
        in_sim = os.environ.get("NIDAR_SIM") == "1"
        if self._sim and not in_sim:
            return "Main Mission (Sim) only runs in the simulator (start_sim.sh) -- select Main Mission"
        if not self._sim and in_sim:
            return "this is the simulator -- select Main Mission (Sim)"
        running = [n for n in _CONFLICTING_NODES if n in self.get_node_names()]
        if running:
            return f"{', '.join(running)} is running and would arm independently -- stop it first"
        return None

    # -- loop -----------------------------------------------------------------

    def _tick(self) -> None:
        snap = self._vehicle.snapshot()
        if snap.fcu_connected and not self._streams_requested:
            self._vehicle.reset_link()
            self._vehicle.request_streams()
            self._streams_requested = True
        elif not snap.fcu_connected:
            self._streams_requested = False
        if snap.fcu_connected:
            self._vehicle.ensure_ekf_origin(*EKF_ORIGIN)
        with self._lock:
            self._run(self._mission.tick(self._mission_now(), snap, self._perception()))

    def _run(self, actions) -> None:
        self._log_change()
        for action in actions:
            if action.kind == "goto":
                self._vehicle.goto(*action.value)  # every tick: not logged
                continue
            value = "" if action.value is None else action.value
            self.get_logger().warning(f"[{_now()}] action: {action.kind} {value}")
            done = lambda ok, detail, kind=action.kind: self._guarded(  # noqa: E731
                f"{kind} result", self._on_result
            )(kind, ok, detail)
            if action.kind == "set_mode":
                self._vehicle.set_mode(action.value, done)
            elif action.kind == "takeoff":
                self._vehicle.takeoff(action.value, done)
            elif action.kind == "set_speed":
                self._vehicle.set_speed(action.value, done)
            elif action.kind == "arm":
                self._vehicle.arm(done)
            elif action.kind == "disarm":
                self._vehicle.disarm(done)

    def _on_result(self, kind: str, ok: bool, detail: str) -> None:
        # Separate call sites per severity (rclpy raises otherwise).
        if ok:
            self.get_logger().info(f"[{_now()}] {kind} OK: {detail}")
        else:
            self.get_logger().error(f"[{_now()}] {kind} FAILED: {detail}")
        with self._lock:
            self._run(self._mission.on_result(self._mission_now(), kind, ok, detail))

    # -- crash safety net -------------------------------------------------------

    def _guarded(self, where: str, callback):
        def wrapper(*args):
            try:
                callback(*args)
            except Exception as exc:  # noqa: BLE001 -- last line of defence
                self.get_logger().error(
                    f"[{_now()}] INTERNAL ERROR in {where}: {exc!r}\n{traceback.format_exc()}"
                )
                try:
                    with self._lock:
                        actions = self._mission.internal_error(
                            self._mission_now(), f"internal error in {where}: {exc!r}"
                        )
                        self._run(actions)
                except Exception as exc2:  # noqa: BLE001
                    self.get_logger().error(
                        f"[{_now()}] could not make the vehicle safe after the error: {exc2!r}{_LAST_RESORT}"
                    )

        return wrapper

    def _log_change(self) -> None:
        current = (self._mission.state, self._mission.detail)
        if current != self._last_logged:
            self._last_logged = current
            self.get_logger().warning(f"[{_now()}] {self._mission_id}: {current[0]} -- {current[1]}")

    def _publish_status(self) -> None:
        with self._lock:
            status = self._mission.status(self._mission_now())
        status["vehicle"] = "sitl" if self._sim else "drone"
        self._status_pub.publish(String(data=json.dumps(status)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--sim", action="store_true", help="Main Mission (Sim): ArduPilot SITL in Gazebo")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = MainMissionNode(sim=args.sim)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

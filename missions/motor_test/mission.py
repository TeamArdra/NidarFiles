#!/usr/bin/env python3
"""NIDAR motor test mission -- ROS 2 node, runs on the Jetson. PROPS OFF.

    radio START (motor_test) -> radio_command_node -> /gcs/mission_select + /gcs/command
      -> command_node (validation) -> /nidar_autonomy/validated_command
      -> THIS NODE -> ArduCopterVehicle.motor_test -> MAVROS -> Pixhawk 6X

Spins each motor in turn with ArduCopter's motor test
(MAV_CMD_DO_MOTOR_TEST): low throttle, a few seconds each, so the
operator can check every motor runs, in the right order and direction.
It does not need a position estimate. ABORT stops the running motor at
once; each motor command also carries its own timeout, so the FCU stops
the motor by itself if this node stops responding.

Decisions live in motor_test_logic.py (pure, unit-tested). This file only
wires it to ROS and publishes the status on /motor_test/status, which
radio_command_node relays to the GCS.

Acts only on a START whose /gcs/mission_select names "motor_test"; STARTs
for other missions are ignored.

Run (after sourcing ROS 2 Humble and ~/nidar_ws/install/setup.bash):

    python3 missions/motor_test/mission.py
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from motor_test_logic import MotorTestConfig, MotorTestMission  # noqa: E402
from nidar_autonomy.ardupilot_vehicle import ArduCopterVehicle  # noqa: E402
from nidar_autonomy.flight_command import FlightCommandClient  # noqa: E402
from nidar_autonomy.mission_selection import MissionSelection  # noqa: E402
from nidar_autonomy.topics import (  # noqa: E402
    MISSION_SELECT_TOPIC,
    MOTOR_TEST_STATUS_TOPIC,
    VALIDATED_COMMAND_TOPIC,
)

MISSION_ID = "motor_test"

# ---------------------------------------------------------------------------
# Motor test parameters. Edit here. PROPS OFF for every run.
# Motors are numbered in ArduPilot's test sequence: 1 = A (front-right on a
# quad X), then clockwise B, C, D.
# ---------------------------------------------------------------------------
CONFIG = MotorTestConfig(
    motor_count=4,
    throttle_pct=8.0,
    per_motor_s=5.0,
    pause_s=1.0,
)

_LAST_RESORT = " -- each motor stops by itself within its timeout; use the RC kill switch if needed"


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


class MotorTestNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_test_mission")
        self._lock = threading.RLock()
        self._mission = MotorTestMission(CONFIG)
        self._vehicle = ArduCopterVehicle(self, FlightCommandClient(self, already_spinning=True))
        # one selection per START (mission_selection.py): a START whose own
        # selection hasn't arrived yet can't reuse an earlier one
        self._selection = MissionSelection()
        self._last_logged = (self._mission.state, self._mission.detail)

        self.create_subscription(String, MISSION_SELECT_TOPIC, self._guarded("selection", self._on_select), 10)
        self.create_subscription(String, VALIDATED_COMMAND_TOPIC, self._guarded("command", self._on_command), 10)
        self._status_pub = self.create_publisher(String, MOTOR_TEST_STATUS_TOPIC, 10)
        self.create_timer(0.1, self._guarded("tick", self._tick))
        self.create_timer(0.5, self._guarded("status", self._publish_status))

        self.get_logger().warning(
            f"[{_now()}] motor test mission ready (PROPS OFF): {CONFIG.motor_count} motors, "
            f"{CONFIG.throttle_pct:.0f}% for {CONFIG.per_motor_s:.0f} s each; waiting for radio START"
        )

    # -- inputs -------------------------------------------------------------

    def _on_select(self, msg: String) -> None:
        try:
            selection = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(f"ignoring malformed mission selection {msg.data!r}")
            return
        if isinstance(selection, dict):
            self._selection.update(selection.get("mission_id"), time.monotonic())

    def _on_command(self, msg: String) -> None:
        now = time.monotonic()
        snap = self._vehicle.snapshot()
        with self._lock:
            if msg.data == "abort":
                if self._mission.state not in ("idle", "complete", "aborted", "failed"):
                    self.get_logger().error(f"[{_now()}] ABORT received (state {self._mission.state})")
                self._run(self._mission.abort(now, snap))
                return
            if msg.data != "start":
                return
            for_us, problem = self._selection.take_for(MISSION_ID, now)
            if not for_us or problem:
                return  # a START for another mission (or without a fresh selection naming this one)
            self.get_logger().warning(f"[{_now()}] START received for {MISSION_ID}")
            self._run(self._mission.start(now, snap))

    # -- loop -----------------------------------------------------------------

    def _tick(self) -> None:
        snap = self._vehicle.snapshot()
        with self._lock:
            self._run(self._mission.tick(time.monotonic(), snap))

    def _run(self, actions) -> None:
        self._log_change()
        for action in actions:
            self.get_logger().warning(
                f"[{_now()}] action: {action.kind} motor {action.motor} "
                f"{action.throttle_pct:.0f}% {action.timeout_s:.1f} s"
            )
            done = lambda ok, detail, a=action: self._guarded(  # noqa: E731
                f"{a.kind} result", self._on_result
            )(a, ok, detail)
            self._vehicle.motor_test(action.motor, action.throttle_pct, action.timeout_s, done)

    def _on_result(self, action, ok: bool, detail: str) -> None:
        # Separate call sites per severity: rclpy raises if one logging
        # call site is used with different severities.
        if ok:
            self.get_logger().info(f"[{_now()}] {action.kind} motor {action.motor} OK: {detail}")
        else:
            self.get_logger().error(f"[{_now()}] {action.kind} motor {action.motor} FAILED: {detail}")
        with self._lock:
            self._run(self._mission.on_result(time.monotonic(), action.kind, action.motor, ok, detail))

    # -- crash safety net -------------------------------------------------------

    def _guarded(self, where: str, callback):
        """Wraps a ROS callback: an unexpected exception (a bug) is logged
        and handled like a mission failure via internal_error() -- the
        node keeps running and the vehicle is made safe -- instead of
        killing the node and leaving the vehicle with nobody in charge."""

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
                            time.monotonic(), f"internal error in {where}: {exc!r}"
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
            self.get_logger().warning(f"[{_now()}] motor test: {current[0]} -- {current[1]}")

    def _publish_status(self) -> None:
        with self._lock:
            status = self._mission.status(time.monotonic())
        self._status_pub.publish(String(data=json.dumps(status)))


def main() -> None:
    rclpy.init()
    node = MotorTestNode()
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

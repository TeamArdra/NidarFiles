#!/usr/bin/env python3
"""Check (and with --apply, set) the Pixhawk parameters a flight needs:
Jetson-loss -> LAND, battery/EKF failsafes -> LAND, flight battery
limits, and the optical-flow / EKF setup. With --rc (an RC safety pilot
on board): RC loss -> LAND, the pilot keeps control if the Jetson dies,
and the mode switch gets Stabilize / AltHold / Loiter. Rules:
nidar_autonomy/flight_param_check.py.

Needs the Jetson stack running (MAVROS connected). Run in a second terminal:

    ~/NidarFiles/scripts/jetson/flight_params.py                # check only (no RC)
    ~/NidarFiles/scripts/jetson/flight_params.py --apply        # set what is missing (disarmed only)
    ~/NidarFiles/scripts/jetson/flight_params.py --rc --apply   # same, with an RC safety pilot

--apply refuses while the vehicle is armed or its state is unknown, sets
only the "FIX" items, reads them back, and prints the result. Some
parameters take effect only after a Pixhawk reboot -- reboot after
applying (Mission Planner, or power-cycle) and run the check again.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from mavros_msgs.msg import State
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nidar_autonomy.flight_param_check import evaluate, names, report

PARAM_NODE = "/mavros/param"
MAVROS_NODE = "/mavros"
MAVROS_DEFAULT_SYSTEM_ID = 1
TIMEOUT_S = 5.0


class Tool(Node):
    def __init__(self) -> None:
        super().__init__("flight_params_tool")
        self.get_fcu = self.create_client(GetParameters, f"{PARAM_NODE}/get_parameters")
        self.set_fcu = self.create_client(SetParameters, f"{PARAM_NODE}/set_parameters")
        self.get_mavros = self.create_client(GetParameters, f"{MAVROS_NODE}/get_parameters")
        self.state = None
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        self.create_subscription(State, "/mavros/state", self._on_state, qos)

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=TIMEOUT_S):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT_S)
        return future.result() if future.done() else None

    def mavros_system_id(self) -> int:
        request = GetParameters.Request()
        request.names = ["system_id"]
        response = self.call(self.get_mavros, request)
        if response and response.values and response.values[0].type == ParameterType.PARAMETER_INTEGER:
            return int(response.values[0].integer_value)
        print(f"note: could not read MAVROS system_id; assuming {MAVROS_DEFAULT_SYSTEM_ID} (MAVROS default)")
        return MAVROS_DEFAULT_SYSTEM_ID

    def read(self, param_names):
        """name -> (value, is_integer) for each parameter MAVROS knows."""
        request = GetParameters.Request()
        request.names = list(param_names)
        response = self.call(self.get_fcu, request)
        out = {}
        if response is None:
            return out
        for name, pv in zip(param_names, response.values):
            if pv.type == ParameterType.PARAMETER_INTEGER:
                out[name] = (float(pv.integer_value), True)
            elif pv.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = (float(pv.double_value), False)
        return out

    def write(self, name: str, value: float, is_integer: bool) -> str:
        pv = ParameterValue()
        if is_integer:
            pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, int(round(value))
        else:
            pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
        request = SetParameters.Request()
        request.parameters = [Parameter(name=name, value=pv)]
        response = self.call(self.set_fcu, request)
        if response is None:
            return "no reply from MAVROS"
        result = response.results[0]
        return "ok" if result.successful else f"refused: {result.reason}"

    def wait_state(self) -> None:
        deadline = time.monotonic() + TIMEOUT_S
        while self.state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--apply", action="store_true", help="set the FIX items (vehicle must be disarmed)")
    parser.add_argument("--rc", action="store_true", help="an RC safety pilot is on board (RC failsafe, mode switch)")
    args = parser.parse_args()

    rclpy.init()
    tool = Tool()
    try:
        tool.wait_state()
        if tool.state is None or not tool.state.connected:
            print("STOP: MAVROS is not connected to the Pixhawk -- start start_jetson.sh first.")
            return 2
        sysid = tool.mavros_system_id()
        wanted = names(sysid, args.rc)
        values = tool.read(wanted)
        if not values:
            print("STOP: could not read parameters from MAVROS (/mavros/param) -- is the parameter list loaded yet?")
            return 2
        results = evaluate({n: v for n, (v, _) in values.items()}, sysid, args.rc)
        print(f"MAVROS system_id = {sysid} (the Pixhawk's SYSID_MYGCS must match)\n")
        print(report(results, args.rc))

        fixes = [r for r in results if r.fix is not None]
        if not args.apply or not fixes:
            return 0 if all(r.ok for r in results) else 1

        tool.wait_state()
        if tool.state is None or tool.state.armed:
            print("\nSTOP: vehicle armed (or state unknown) -- nothing written.")
            return 2
        print("\nApplying:")
        for r in fixes:
            outcome = tool.write(r.name, r.fix, values[r.name][1])
            print(f"  {r.name}: {r.value:g} -> {r.fix:g}  {outcome}")
        print("\nRe-reading:\n")
        values = tool.read(wanted)
        results = evaluate({n: v for n, (v, _) in values.items()}, sysid, args.rc)
        print(report(results, args.rc))
        print("\nReboot the Pixhawk now so every change takes effect, then run this check again.")
        return 0 if all(r.ok for r in results) else 1
    finally:
        tool.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

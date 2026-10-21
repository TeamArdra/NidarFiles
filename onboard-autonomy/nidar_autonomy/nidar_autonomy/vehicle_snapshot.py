"""A plain, rclpy-free snapshot of the vehicle state a mission decides on
-- produced by ardupilot_vehicle.ArduCopterVehicle.snapshot(), consumed
by mission logic (e.g. missions/hover/hover_logic.py) so that logic is
unit-testable without ROS. Ages are seconds since the last message;
None means "never received" -- missions must treat that as unknown, not
as a default value."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class VehicleSnapshot:
    fcu_connected: bool
    state_age_s: Optional[float]
    armed: Optional[bool]
    mode: Optional[str]
    position: Optional[Tuple[float, float, float]]  # MAVROS local ENU, metres
    position_age_s: Optional[float]
    battery_voltage_v: Optional[float]
    battery_age_s: Optional[float]
    # Seconds since the FCU last reported its EKF origin (GPS_GLOBAL_ORIGIN);
    # None = never on this run. Without an origin ArduCopter refuses a
    # GUIDED takeoff (no GPS indoors: the Jetson sets it, ardupilot_vehicle).
    ekf_origin_age_s: Optional[float] = None
    # Heading of the local position (radians, ENU: 0 = +x/east, counter-
    # clockwise positive), from the same /mavros/local_position/pose message
    # as `position`. None until that message arrives.
    yaw: Optional[float] = None

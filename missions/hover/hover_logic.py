"""Hover mission decision logic -- pure, no rclpy, driven by explicit
start()/abort()/tick()/on_result() calls with a monotonic `now`, so every
branch is unit-testable (test_hover_logic.py). mission.py is the ROS node
that feeds it VehicleSnapshots and executes the Actions it returns.

ArduCopter 4.6.3 sequence (no PX4 OFFBOARD, no setpoint stream):

    preflight checks -> GUIDED -> ARM -> TAKEOFF(h) -> reach ~h
      -> hold for T (GUIDED holds position by itself) -> LAND
      -> ArduCopter disarms itself after touchdown -> complete

Safety rules encoded here:
  - Preflight refuses to start (nothing is armed) without a connected
    FCU, a disarmed vehicle, a fresh local position estimate (the indoor
    position source) and, if configured, enough battery voltage.
  - In the air every failure ends in LAND mode -- never a mid-air disarm.
  - ABORT: before arming -> stop; while arming -> disarm (vehicle is still
    on the ground); airborne -> LAND.
  - If the flight mode changes to anything we didn't request (pilot on
    the RC transmitter), the mission stops commanding immediately and
    never fights the pilot. A switch to LAND by the FCU (e.g. a failsafe)
    is followed, not overridden.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from nidar_autonomy.vehicle_snapshot import VehicleSnapshot

GUIDED = "GUIDED"
LAND = "LAND"

IDLE = "idle"
PREFLIGHT = "preflight"
SETTING_GUIDED = "setting_guided"
ARMING = "arming"
TAKING_OFF = "taking_off"
HOVERING = "hovering"
LANDING = "landing"
COMPLETE = "complete"
ABORTED = "aborted"
FAILED = "failed"
PILOT_OVERRIDE = "pilot_override"

TERMINAL_STATES = frozenset({IDLE, COMPLETE, ABORTED, FAILED, PILOT_OVERRIDE})


@dataclass(frozen=True)
class HoverConfig:
    """Flight parameters. Defaults are deliberately conservative for a
    first indoor test; mission.py sets the values actually flown."""

    takeoff_altitude_m: float = 0.5
    hold_duration_s: float = 10.0
    altitude_reached_fraction: float = 0.8  # "reached" = climbed this fraction of the target
    climb_timeout_s: float = 15.0
    max_altitude_margin_m: float = 0.5  # LAND if higher than target + this
    max_horizontal_drift_m: float = 0.75  # LAND if this far from the takeoff point
    position_stale_s: float = 1.0
    state_stale_s: float = 3.0  # /mavros/state arrives at ~1 Hz
    mode_confirm_timeout_s: float = 5.0
    arm_timeout_s: float = 15.0
    takeoff_ack_timeout_s: float = 5.0
    land_request_interval_s: float = 2.0
    land_timeout_s: float = 60.0
    min_battery_voltage_v: Optional[float] = None
    battery_stale_s: float = 5.0
    origin_stale_s: float = 10.0  # EKF origin must have been reported this recently


@dataclass(frozen=True)
class Action:
    kind: str  # "set_mode" | "arm" | "disarm" | "takeoff"
    value: object = None


class HoverMission:
    def __init__(self, config: HoverConfig) -> None:
        self.config = config
        self.state = IDLE
        self.detail = "waiting for START"
        self._outcome = COMPLETE
        self._snap: Optional[VehicleSnapshot] = None
        self._ground: Optional[Tuple[float, float, float]] = None
        self._deadline = 0.0
        self._climb_deadline = 0.0
        self._takeoff_acked = False
        self._hold_until = 0.0
        self._hover_started = 0.0
        self._landing_started = 0.0
        self._last_land_request = 0.0
        self._land_timeout_reported = False

    # -- operator commands ----------------------------------------------------

    def start(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        self._snap = snap
        if self.state not in TERMINAL_STATES:
            self.detail = f"START ignored: already {self.state}"
            return []
        self.__init__(self.config)
        self._snap = snap
        self.state = PREFLIGHT
        problems = self._preflight_problems(snap)
        if problems:
            return self._fail("preflight failed: " + "; ".join(problems))
        self._ground = snap.position
        if snap.mode == GUIDED:
            return self._request_arm(now)
        self.state = SETTING_GUIDED
        self.detail = "requesting GUIDED"
        self._deadline = now + self.config.mode_confirm_timeout_s
        return [Action("set_mode", GUIDED)]

    def reject_start(self, reason: str) -> None:
        """START refused by the node before reaching the mission (e.g. a
        conflicting node is running). Nothing was commanded."""
        if self.state in TERMINAL_STATES:
            self.state = FAILED
            self.detail = reason

    def abort(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        self._snap = snap
        if self.state in TERMINAL_STATES:
            self.detail = f"ABORT received while {self.state}: nothing to stop"
            return []
        if self.state in (PREFLIGHT, SETTING_GUIDED):
            self.state = ABORTED
            self.detail = "operator ABORT before arming"
            return []
        if self.state == ARMING:
            self.state = ABORTED
            self.detail = "operator ABORT while arming -- disarming on the ground"
            return [Action("disarm")]
        if self.state in (TAKING_OFF, HOVERING):
            return self._land(now, "operator ABORT -- landing", ABORTED)
        # LANDING: already descending; record that the operator aborted.
        if self._outcome == COMPLETE:
            self._outcome = ABORTED
        self.detail = "operator ABORT -- already landing"
        return []

    def internal_error(self, now: float, reason: str) -> List[Action]:
        """The node hit an unexpected error (a bug). Treated as a failure:
        nothing new is started; on the ground before takeoff -> disarm;
        airborne -> LAND. Safe to call repeatedly."""
        if self.state in TERMINAL_STATES:
            self.detail = reason
            return []
        if self.state in (PREFLIGHT, SETTING_GUIDED):
            self.state = FAILED
            self.detail = reason
            return []
        if self.state == ARMING:
            self.state = FAILED
            self.detail = f"{reason} -- disarming on the ground"
            return [Action("disarm")]
        if self.state in (TAKING_OFF, HOVERING):
            return self._land(now, f"{reason} -- landing", FAILED)
        self._outcome = FAILED  # LANDING: keep descending
        self.detail = f"{reason} -- already landing"
        return []

    # -- periodic ---------------------------------------------------------------

    def tick(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        self._snap = snap
        if self.state == SETTING_GUIDED:
            if not self._link_ok(snap):
                return self._fail("FCU link lost while setting GUIDED")
            if snap.mode == GUIDED:
                return self._request_arm(now)
            if now > self._deadline:
                return self._fail(f"GUIDED not confirmed within {self.config.mode_confirm_timeout_s:.0f}s")
            return []

        if self.state == ARMING:
            if self._pilot_switched(snap):
                return self._pilot_override(f"flight mode changed to {snap.mode} while arming -- pilot has control")
            if now > self._deadline:
                self._fail(f"ARM not confirmed within {self.config.arm_timeout_s:.0f}s")
                return [Action("disarm")]
            return []

        if self.state in (TAKING_OFF, HOVERING):
            actions = self._airborne_watchdog(now, snap)
            if actions is not None:
                return actions
            altitude = self._altitude(snap)
            if self.state == TAKING_OFF:
                if not self._takeoff_acked and now > self._deadline:
                    return self._land(now, "TAKEOFF not acknowledged -- landing", FAILED)
                if altitude is not None and altitude >= self.config.altitude_reached_fraction * self.config.takeoff_altitude_m:
                    self.state = HOVERING
                    self._hover_started = now
                    self._hold_until = now + self.config.hold_duration_s
                    self.detail = f"hovering at {altitude:.2f} m for {self.config.hold_duration_s:.0f}s"
                    return []
                if now > self._climb_deadline:
                    return self._land(now, "target altitude not reached in time -- landing", FAILED)
                return []
            if now >= self._hold_until:
                return self._land(now, "hover complete -- landing", COMPLETE)
            return []

        if self.state == LANDING:
            return self._tick_landing(now, snap)
        return []

    def on_result(self, now: float, kind: str, ok: bool, detail: str) -> List[Action]:
        """Outcome of an Action the node executed."""
        if kind == "arm":
            if self.state == PILOT_OVERRIDE:
                return []  # the pilot flipped the mode switch while arming: theirs to fly or disarm
            if self.state != ARMING:
                # e.g. ABORT arrived while the arm request was in flight
                return [Action("disarm")] if ok else []
            if not ok:
                self._fail(f"ARM failed: {detail}")
                return [Action("disarm")]  # defensive: FCU/state mismatch is possible
            if self._pilot_switched(self._snap):
                return self._pilot_override(f"flight mode changed to {self._snap.mode} while arming -- pilot has control")
            if self._snap is not None and self._snap.position is not None:
                self._ground = self._snap.position
            self.state = TAKING_OFF
            self.detail = f"armed -- taking off to {self.config.takeoff_altitude_m:.2f} m"
            self._takeoff_acked = False
            self._deadline = now + self.config.takeoff_ack_timeout_s
            self._climb_deadline = now + self.config.climb_timeout_s
            return [Action("takeoff", self.config.takeoff_altitude_m)]

        if kind == "takeoff" and self.state == TAKING_OFF:
            if ok:
                self._takeoff_acked = True
                return []
            if self._pilot_switched(self._snap):
                # rejected because the pilot left GUIDED: don't command LAND under them
                return self._pilot_override(f"TAKEOFF rejected in {self._snap.mode} -- pilot has control")
            return self._land(now, f"TAKEOFF rejected ({detail}) -- landing", FAILED)

        if kind == "set_mode" and not ok:
            if self.state == SETTING_GUIDED:
                return self._fail(f"GUIDED request rejected: {detail}")
            if self.state == LANDING:
                self.detail = f"LAND request failed ({detail}) -- retrying"
            return []

        if kind == "disarm" and not ok:
            self.detail = f"DISARM FAILED ({detail}) -- USE THE KILL SWITCH"
        return []

    # -- status -----------------------------------------------------------------

    def status(self, now: float) -> dict:
        snap = self._snap
        altitude = self._altitude(snap) if snap is not None else None
        return {
            "scenario": "hover",
            "state": self.state,
            "detail": self.detail,
            "target_altitude_m": self.config.takeoff_altitude_m,
            "current_altitude_m": None if altitude is None else round(altitude, 2),
            "current_position": None if snap is None or snap.position is None else [round(v, 2) for v in snap.position],
            "duration_s": self.config.hold_duration_s,
            "elapsed_hover_s": (
                round(min(now - self._hover_started, self.config.hold_duration_s), 1)
                if self.state == HOVERING
                else None
            ),
            "armed": None if snap is None else snap.armed,
            "flight_mode": None if snap is None else snap.mode,
            "execution_mode": "real",
        }

    # -- internals ----------------------------------------------------------------

    def _preflight_problems(self, snap: VehicleSnapshot) -> List[str]:
        c = self.config
        problems = []
        if not self._link_ok(snap):
            problems.append("FCU not connected (/mavros/state stale or connected=false)")
        if snap.armed is not False:
            problems.append("vehicle already armed or armed state unknown")
        if snap.ekf_origin_age_s is None or snap.ekf_origin_age_s > c.origin_stale_s:
            problems.append(
                "EKF origin not set (the Jetson sets it at startup -- see hover_mission.log; "
                "ArduCopter refuses a GUIDED takeoff without it)"
            )
        if snap.position is None or snap.position_age_s is None or snap.position_age_s > c.position_stale_s:
            problems.append(
                "no fresh local position -- the EKF has no indoor position estimate "
                "(check optical flow / rangefinder / EKF3 source configuration)"
            )
        if c.min_battery_voltage_v is not None:
            if snap.battery_voltage_v is None or snap.battery_age_s is None or snap.battery_age_s > c.battery_stale_s:
                problems.append("no battery voltage reading")
            elif snap.battery_voltage_v < c.min_battery_voltage_v:
                problems.append(
                    f"battery {snap.battery_voltage_v:.2f} V below minimum {c.min_battery_voltage_v:.2f} V"
                )
        return problems

    def _link_ok(self, snap: VehicleSnapshot) -> bool:
        return (
            snap.fcu_connected
            and snap.state_age_s is not None
            and snap.state_age_s <= self.config.state_stale_s
        )

    def _request_arm(self, now: float) -> List[Action]:
        self.state = ARMING
        self.detail = "GUIDED confirmed -- arming"
        self._deadline = now + self.config.arm_timeout_s
        return [Action("arm")]

    def _airborne_watchdog(self, now: float, snap: VehicleSnapshot) -> Optional[List[Action]]:
        c = self.config
        if not self._link_ok(snap):
            return self._land(now, "FCU/MAVROS link lost -- requesting LAND", FAILED)
        if snap.armed is False:
            return self._fail("vehicle disarmed unexpectedly while airborne")
        if snap.mode == LAND:
            self.state = LANDING
            self._outcome = FAILED
            self._landing_started = now
            self._last_land_request = now
            self.detail = "FCU switched to LAND (failsafe?) -- following it"
            return []
        if snap.mode != GUIDED:
            self.state = PILOT_OVERRIDE
            self.detail = f"flight mode changed to {snap.mode} -- mission stopped commanding (pilot has control)"
            return []
        # No EKF-origin check here on purpose: the FCU keeps its origin until
        # it reboots, so a stale origin *report* in flight only means the
        # 1 Hz GPS_GLOBAL_ORIGIN message stopped -- landing on that would be a
        # false abort. The origin is required before take-off (preflight); a
        # real EKF problem in flight shows as stale position, checked next.
        if snap.position is None or snap.position_age_s is None or snap.position_age_s > c.position_stale_s:
            return self._land(now, "local position stale -- landing", FAILED)
        altitude = self._altitude(snap)
        if altitude is not None and altitude > c.takeoff_altitude_m + c.max_altitude_margin_m:
            return self._land(now, f"altitude {altitude:.2f} m above limit -- landing", FAILED)
        drift = self._drift(snap)
        if drift is not None and drift > c.max_horizontal_drift_m:
            return self._land(now, f"horizontal drift {drift:.2f} m above limit -- landing", FAILED)
        return None

    def _tick_landing(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        if snap.armed is False and self._link_ok(snap):
            self.state = self._outcome
            self.detail = {
                COMPLETE: "landed and disarmed -- hover complete",
                ABORTED: "landed and disarmed after ABORT",
            }.get(self._outcome, f"landed and disarmed ({self.detail})")
            return []
        if now - self._landing_started > self.config.land_timeout_s and not self._land_timeout_reported:
            self._land_timeout_reported = True
            self.detail = "still not disarmed after LAND timeout -- check the vehicle"
        if snap.mode == LAND:
            return []
        if snap.mode == GUIDED or snap.mode is None:
            if now - self._last_land_request >= self.config.land_request_interval_s:
                self._last_land_request = now
                return [Action("set_mode", LAND)]
            return []
        self.state = PILOT_OVERRIDE
        self.detail = f"flight mode changed to {snap.mode} during landing -- pilot has control"
        return []

    def _land(self, now: float, reason: str, outcome: str) -> List[Action]:
        self.state = LANDING
        self._outcome = outcome
        self.detail = reason
        self._landing_started = now
        self._last_land_request = now
        return [Action("set_mode", LAND)]

    @staticmethod
    def _pilot_switched(snap: Optional[VehicleSnapshot]) -> bool:
        """A known mode other than GUIDED/LAND: someone on the RC switch."""
        return snap is not None and snap.mode not in (None, GUIDED, LAND)

    def _pilot_override(self, reason: str) -> List[Action]:
        self.state = PILOT_OVERRIDE
        self.detail = reason
        return []

    def _fail(self, reason: str) -> List[Action]:
        self.state = FAILED
        self.detail = reason
        return []

    def _altitude(self, snap: VehicleSnapshot) -> Optional[float]:
        if snap.position is None or self._ground is None:
            return None
        return snap.position[2] - self._ground[2]

    def _drift(self, snap: VehicleSnapshot) -> Optional[float]:
        if snap.position is None or self._ground is None:
            return None
        return math.hypot(snap.position[0] - self._ground[0], snap.position[1] - self._ground[1])

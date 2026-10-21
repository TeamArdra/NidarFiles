"""Main Mission decision logic -- pure, no rclpy (same pattern as
missions/hover/hover_logic.py): driven by start()/abort()/tick()/on_result()
with a monotonic `now`, returns Actions for mission.py to execute.

The NIDAR AirMouse flight, ArduCopter 4.6.3 GUIDED (no PX4 OFFBOARD):

    preflight -> GUIDED -> ARM -> TAKEOFF(h) -> reach ~h
      -> record the entry (SLAM pose) -> EXPLORING: fly to frontiers until
         nothing reachable is left (or the time budget is used)
      -> RETURNING: fly back to the entry
      -> LAND at the entry -> ArduCopter disarms after touchdown -> complete

The same logic flies the real drone (Main Mission) and ArduPilot SITL in
Gazebo (Main Mission (Sim)); only MainConfig values differ (mission.py).

Two position frames: the SLAM map (Cartographer: plans, frontiers, the
entry) and the FCU's local frame (setpoints). They drift apart (optical-flow
EKF vs scan matching), so the map->local transform is re-estimated every
tick from both live poses (geometry.RigidTransform2D, smoothed) -- the
approach gps_denied's path follower used against PX4.

Safety rules (on top of hover's, which are kept as-is):
  - Preflight also needs a fresh map and SLAM pose.
  - In the air every failure ends in LAND; never a mid-air disarm.
  - ABORT: before arming -> stop; while arming -> disarm; airborne -> LAND
    where it is (the rulebook's abort stops, it does not recover).
  - Pilot takes the RC mode switch -> stop commanding at once.
  - SLAM pose lost -> hold position; still lost after slam_lost_land_s -> LAND.
  - Low battery, geofence (too far from the entry) or the time budget ->
    RETURNING; critical battery -> LAND where it is.
  - No path home, or the return takes too long -> LAND where it is.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from explore_logic import (
    EntryPose,
    ExploreConfig,
    Explorer,
    FollowConfig,
    PathFollower,
    PlanningGrid,
    path_length,
    plan_path,
)
from nidar_autonomy.geometry import RigidTransform2D, wrap
from nidar_autonomy.vehicle_snapshot import VehicleSnapshot

GUIDED = "GUIDED"
LAND = "LAND"

IDLE = "idle"
PREFLIGHT = "preflight"
SETTING_GUIDED = "setting_guided"
ARMING = "arming"
TAKING_OFF = "taking_off"
EXPLORING = "exploring"
RETURNING = "returning"
LANDING = "landing"
COMPLETE = "complete"
ABORTED = "aborted"
FAILED = "failed"
PILOT_OVERRIDE = "pilot_override"

TERMINAL_STATES = frozenset({IDLE, COMPLETE, ABORTED, FAILED, PILOT_OVERRIDE})
FLYING_STATES = frozenset({TAKING_OFF, EXPLORING, RETURNING})


@dataclass(frozen=True)
class MainConfig:
    takeoff_altitude_m: float = 1.0
    cruise_speed_mps: float = 0.4  # DO_CHANGE_SPEED after takeoff (GUIDED)
    explore_budget_s: float = 600.0  # then return, whatever is left (15 min bonus)
    return_timeout_s: float = 240.0
    entry_tolerance_m: float = 0.4  # "back at the entry" -> LAND
    replan_period_s: float = 3.0  # while following a path
    idle_replan_s: float = 1.0  # while holding without one (the map updates ~1 Hz)
    exploration_done_cycles: int = 3  # replans in a row with nothing reachable
    map_ready_timeout_s: float = 30.0  # no free space around the drone yet -> wait this long, then land
    geofence_m: float = 20.0  # from the entry, horizontal
    max_altitude_margin_m: float = 0.6
    altitude_reached_fraction: float = 0.8
    climb_timeout_s: float = 20.0
    position_stale_s: float = 1.0
    state_stale_s: float = 3.0
    map_stale_s: float = 5.0
    slam_stale_s: float = 1.5
    slam_lost_land_s: float = 6.0
    max_map_local_jump_m: float = 1.5  # map->local transform jump treated as SLAM failure
    mode_confirm_timeout_s: float = 5.0
    arm_timeout_s: float = 15.0
    takeoff_ack_timeout_s: float = 5.0
    land_request_interval_s: float = 2.0
    land_timeout_s: float = 90.0
    origin_stale_s: float = 10.0
    min_battery_voltage_v: Optional[float] = None  # preflight
    low_battery_v: Optional[float] = None  # in flight -> return
    critical_battery_v: Optional[float] = None  # in flight -> land now
    battery_stale_s: float = 5.0
    explore: ExploreConfig = field(default_factory=ExploreConfig)
    follow: FollowConfig = field(default_factory=FollowConfig)


@dataclass(frozen=True)
class Perception:
    """What the mission sees besides the FCU. `map_grid` is the /map dict
    (frontier_detector shape); `slam_pose` = (x, y, yaw) of the LiDAR in the
    map frame (TF map -> laser). Ages in seconds, None = never."""

    map_grid: Optional[dict] = None
    map_age_s: Optional[float] = None
    slam_pose: Optional[Tuple[float, float, float]] = None
    slam_age_s: Optional[float] = None


@dataclass(frozen=True)
class Action:
    kind: str  # "set_mode" | "arm" | "disarm" | "takeoff" | "set_speed" | "goto"
    value: object = None  # goto: (x, y, z, yaw) in the FCU's local frame


class MainMission:
    def __init__(self, config: MainConfig, mission_id: str = "main_mission") -> None:
        self.config = config
        self.mission_id = mission_id
        self.state = IDLE
        self.detail = "waiting for START"
        self._outcome = COMPLETE
        self._snap: Optional[VehicleSnapshot] = None
        self._seen = Perception()
        self._ground: Optional[Tuple[float, float, float]] = None
        self._deadline = 0.0
        self._climb_deadline = 0.0
        self._takeoff_acked = False
        self._landing_started = 0.0
        self._last_land_request = 0.0
        self._land_timeout_reported = False
        self._started_at: Optional[float] = None
        self._explore_started: Optional[float] = None
        self._return_started: Optional[float] = None
        self._entry: Optional[EntryPose] = None
        self._transform = RigidTransform2D(smoothing=0.05)
        self._explorer = Explorer(config.explore)
        self._follower = PathFollower(config.follow)
        self._last_plan = -math.inf
        self._empty_cycles = 0
        self._map_wait_since: Optional[float] = None
        self._slam_lost_since: Optional[float] = None
        self._hold: Optional[Tuple[float, float, float]] = None  # local x, y, yaw while holding
        self._return_reason = ""
        self._shown_goal: Optional[Tuple[float, float]] = None
        self._grid: Optional[PlanningGrid] = None
        self._grid_source: Optional[dict] = None  # the /map dict it was built from (kept, so `is` is safe)

    # -- operator commands ----------------------------------------------------

    def start(self, now: float, snap: VehicleSnapshot, seen: Perception) -> List[Action]:
        self._snap = snap
        if self.state not in TERMINAL_STATES:
            self.detail = f"START ignored: already {self.state}"
            return []
        self.__init__(self.config, self.mission_id)
        self._snap, self._seen = snap, seen
        self.state = PREFLIGHT
        problems = self._preflight_problems(snap, seen)
        if problems:
            return self._fail("preflight failed: " + "; ".join(problems))
        self._ground = snap.position
        self._started_at = now
        if snap.mode == GUIDED:
            return self._request_arm(now)
        self.state = SETTING_GUIDED
        self.detail = "requesting GUIDED"
        self._deadline = now + self.config.mode_confirm_timeout_s
        return [Action("set_mode", GUIDED)]

    def reject_start(self, reason: str) -> None:
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
        if self.state in FLYING_STATES:
            return self._land(now, "operator ABORT -- landing where it is", ABORTED)
        if self._outcome == COMPLETE:
            self._outcome = ABORTED
        self.detail = "operator ABORT -- already landing"
        return []

    def internal_error(self, now: float, reason: str) -> List[Action]:
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
        if self.state in FLYING_STATES:
            return self._land(now, f"{reason} -- landing", FAILED)
        self._outcome = FAILED
        self.detail = f"{reason} -- already landing"
        return []

    # -- periodic ---------------------------------------------------------------

    def tick(self, now: float, snap: VehicleSnapshot, seen: Perception) -> List[Action]:
        self._snap, self._seen = snap, seen
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

        if self.state in FLYING_STATES:
            actions = self._airborne_watchdog(now, snap)
            if actions is not None:
                return actions
            self._update_transform(seen, snap)
            if self.state == TAKING_OFF:
                return self._tick_takeoff(now, snap, seen)
            return self._tick_navigate(now, snap, seen)

        if self.state == LANDING:
            return self._tick_landing(now, snap)
        return []

    def on_result(self, now: float, kind: str, ok: bool, detail: str) -> List[Action]:
        if kind == "arm":
            if self.state == PILOT_OVERRIDE:
                return []
            if self.state != ARMING:
                return [Action("disarm")] if ok else []
            if not ok:
                self._fail(f"ARM failed: {detail}")
                return [Action("disarm")]
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
                return self._pilot_override(f"TAKEOFF rejected in {self._snap.mode} -- pilot has control")
            return self._land(now, f"TAKEOFF rejected ({detail}) -- landing", FAILED)

        if kind == "set_mode" and not ok:
            if self.state == SETTING_GUIDED:
                return self._fail(f"GUIDED request rejected: {detail}")
            if self.state == LANDING:
                self.detail = f"LAND request failed ({detail}) -- retrying"
            return []

        if kind == "set_speed" and not ok:
            # ArduCopter keeps WPNAV_SPEED: not fatal, but say so.
            self.detail = f"{self.detail} (speed limit not set: {detail} -- WPNAV_SPEED applies)"
            return []

        if kind == "disarm" and not ok:
            self.detail = f"DISARM FAILED ({detail}) -- USE THE KILL SWITCH"
        return []

    # -- status -----------------------------------------------------------------

    def status(self, now: float) -> dict:
        snap = self._snap
        altitude = self._altitude(snap) if snap is not None else None
        pose = self._seen.slam_pose
        to_entry = None
        if self._entry is not None and pose is not None:
            to_entry = round(math.hypot(pose[0] - self._entry.x, pose[1] - self._entry.y), 2)
        return {
            "scenario": self.mission_id,
            "state": self.state,
            "detail": self.detail,
            "target_altitude_m": self.config.takeoff_altitude_m,
            "current_altitude_m": None if altitude is None else round(altitude, 2),
            "current_position": None if snap is None or snap.position is None else [round(v, 2) for v in snap.position],
            "mission_elapsed_s": None if self._started_at is None or self.state in TERMINAL_STATES else round(now - self._started_at, 1),
            "explore_elapsed_s": (
                round(now - self._explore_started, 1) if self._explore_started is not None and self.state == EXPLORING else None
            ),
            "explore_budget_s": self.config.explore_budget_s,
            "frontier_count": self._explorer.frontier_count,
            "goal": None if self._explorer.goal is None else [round(v, 2) for v in self._explorer.goal],
            "distance_to_entry_m": to_entry,
            "armed": None if snap is None else snap.armed,
            "flight_mode": None if snap is None else snap.mode,
            "execution_mode": "real",
        }

    # -- internals: flight phases ------------------------------------------------

    def _tick_takeoff(self, now: float, snap: VehicleSnapshot, seen: Perception) -> List[Action]:
        if not self._takeoff_acked and now > self._deadline:
            return self._land(now, "TAKEOFF not acknowledged -- landing", FAILED)
        altitude = self._altitude(snap)
        if altitude is not None and altitude >= self.config.altitude_reached_fraction * self.config.takeoff_altitude_m:
            if not self._slam_ok(seen):
                return self._land(now, "no SLAM pose at take-off altitude -- landing", FAILED)
            x, y, yaw = seen.slam_pose
            self._entry = EntryPose(x, y, yaw)
            self.state = EXPLORING
            self._explore_started = now
            self.detail = f"at {altitude:.2f} m -- entry recorded, exploring"
            return [Action("set_speed", self.config.cruise_speed_mps)] + self._hold_here(snap)
        if now > self._climb_deadline:
            return self._land(now, "target altitude not reached in time -- landing", FAILED)
        return []

    def _tick_navigate(self, now: float, snap: VehicleSnapshot, seen: Perception) -> List[Action]:
        c = self.config
        # SLAM pose lost: hold where the FCU thinks it is, land if it stays lost
        if not self._slam_ok(seen) or not self._map_ok(seen):
            if self._slam_lost_since is None:
                self._slam_lost_since = now
                self._hold = None
            if now - self._slam_lost_since > c.slam_lost_land_s:
                return self._land(now, "SLAM pose/map lost -- landing", FAILED)
            self.detail = "SLAM pose/map lost -- holding position"
            return self._hold_here(snap)
        if self._slam_lost_since is not None:
            self._slam_lost_since = None
            self._hold = None
            self._follower.clear()  # replan from the recovered pose
            self._last_plan = -math.inf

        x, y, yaw = seen.slam_pose
        self._explorer.record_pose(x, y)
        entry = self._entry

        # leaving the arena / out of time / low battery -> go home
        if self.state == EXPLORING:
            reason = None
            if math.hypot(x - entry.x, y - entry.y) > c.geofence_m:
                reason = f"geofence: {c.geofence_m:.0f} m from the entry"
            elif self._battery_below(snap, c.low_battery_v):
                reason = f"low battery ({snap.battery_voltage_v:.2f} V)"
            elif now - self._explore_started >= c.explore_budget_s:
                reason = f"exploration time budget ({c.explore_budget_s:.0f} s) used"
            if reason:
                return self._begin_return(now, reason)

        grid = self._planning_grid(seen.map_grid)
        due = c.replan_period_s if self._follower.path else c.idle_replan_s
        if now - self._last_plan >= due:
            self._last_plan = now
            if self.state == EXPLORING and grid.nearest_free((x, y), c.explore.snap_radius_m) is None:
                # Nothing confirmed free around the drone yet (a fresh map):
                # that is "map not ready", not "map complete".
                if self._map_wait_since is None:
                    self._map_wait_since = now
                if now - self._map_wait_since > c.map_ready_timeout_s:
                    return self._land(now, "no free space on the map around the drone -- landing", FAILED)
                self.detail = "waiting for the map to show free space around the drone"
                self._follower.clear()
            elif self.state == EXPLORING:
                self._map_wait_since = None
                choice = self._explorer.choose(seen.map_grid, (x, y), entry, grid)
                if choice is None:
                    self._empty_cycles += 1
                    if self._empty_cycles >= c.exploration_done_cycles:
                        return self._begin_return(now, "no reachable frontier left -- map complete")
                    self.detail = f"no reachable frontier ({self._empty_cycles}/{c.exploration_done_cycles}) -- holding"
                    self._follower.clear()
                else:
                    self._empty_cycles = 0
                    self._follower.set_path(choice.path, now)
                    shown = self._shown_goal
                    if shown is None or math.hypot(choice.goal[0] - shown[0], choice.goal[1] - shown[1]) > 0.5:
                        # the detail line goes out over the radio: only on a new goal
                        self._shown_goal = choice.goal
                        self.detail = (
                            f"exploring: goal ({choice.goal[0]:.1f}, {choice.goal[1]:.1f}), "
                            f"{path_length(choice.path):.1f} m, {choice.frontier_count} frontier(s)"
                        )
            elif math.hypot(x - entry.x, y - entry.y) > c.entry_tolerance_m:  # RETURNING: path home on the newest map
                path = plan_path(grid, (x, y), (entry.x, entry.y), c.explore.snap_radius_m)
                if path is None:
                    if not self._follower.path:
                        return self._land(now, "no path back to the entry -- landing where it is", FAILED)
                else:
                    self._follower.set_path(path, now)

        if self.state == RETURNING:
            if math.hypot(x - entry.x, y - entry.y) <= c.entry_tolerance_m:
                return self._land(now, f"back at the entry ({self._return_reason}) -- landing", COMPLETE)
            if now - self._return_started > c.return_timeout_s:
                return self._land(now, "return took too long -- landing where it is", FAILED)

        if not self._follower.path:
            return self._hold_here(snap)
        cmd = self._follower.step(now, (x, y), yaw, grid.wall_points_near((x, y), c.follow.repulsion_radius_m))
        if cmd.reached:
            if self.state == EXPLORING and self._explorer.goal is not None:
                self._explorer.arrived(self._explorer.goal)
            self._follower.clear()
            self._last_plan = -math.inf  # next goal at once
            return self._hold_here(snap)
        if cmd.stuck:
            if self.state == EXPLORING and self._explorer.goal is not None:
                self._explorer.fail(self._explorer.goal)
            self._follower.clear()
            self._last_plan = -math.inf
            self.detail = "no progress -- trying another way"
            return self._hold_here(snap)
        self._hold = None
        return [self._goto_map(cmd.target, cmd.yaw)]

    def _begin_return(self, now: float, reason: str) -> List[Action]:
        self.state = RETURNING
        self._return_started = now
        self._return_reason = reason
        self._follower.clear()
        self._last_plan = -math.inf
        self._explorer.goal = None
        self.detail = f"{reason} -- returning to the entry"
        return self._hold_here(self._snap)

    def _planning_grid(self, map_grid: dict) -> PlanningGrid:
        """Rebuilt only when a new /map arrives (~1 Hz), not every tick."""
        if self._grid is None or self._grid_source is not map_grid:
            self._grid = PlanningGrid(map_grid, self.config.explore, self._entry)
            self._grid_source = map_grid
        return self._grid

    # -- internals: frames --------------------------------------------------------

    def _update_transform(self, seen: Perception, snap: VehicleSnapshot) -> None:
        if not self._slam_ok(seen) or snap.position is None or snap.yaw is None:
            return
        mx, my, myaw = seen.slam_pose
        lx, ly, lyaw = snap.position[0], snap.position[1], snap.yaw
        raw = RigidTransform2D()
        raw.update(mx, my, myaw, lx, ly, lyaw)  # this sample alone
        tr = self._transform
        if tr.initialized and (
            math.hypot(raw.x - tr.x, raw.y - tr.y) > self.config.max_map_local_jump_m
            or abs(wrap(raw.yaw - tr.yaw)) > math.radians(30)
        ):
            # one pose source jumped (e.g. a SLAM loop closure moved the map):
            # restart from this sample instead of averaging the jump in slowly
            self._transform = raw
            raw.smoothing = tr.smoothing
            return
        tr.update(mx, my, myaw, lx, ly, lyaw)

    def _goto_map(self, target, yaw: float) -> Action:
        lx, ly, lyaw = self._transform.apply(target[0], target[1], yaw)
        return Action("goto", (lx, ly, self._target_z(), lyaw))

    def _hold_here(self, snap: Optional[VehicleSnapshot]) -> List[Action]:
        """Keep the FCU's target where the vehicle is now (latched on the
        first call, so holding doesn't creep with position noise)."""
        if snap is None or snap.position is None:
            return []
        if self._hold is None:
            self._hold = (snap.position[0], snap.position[1], snap.yaw if snap.yaw is not None else 0.0)
        hx, hy, hyaw = self._hold
        return [Action("goto", (hx, hy, self._target_z(), hyaw))]

    def _target_z(self) -> float:
        ground = self._ground[2] if self._ground is not None else 0.0
        return ground + self.config.takeoff_altitude_m

    # -- internals: checks --------------------------------------------------------

    def _preflight_problems(self, snap: VehicleSnapshot, seen: Perception) -> List[str]:
        c = self.config
        problems = []
        if not self._link_ok(snap):
            problems.append("FCU not connected (/mavros/state stale or connected=false)")
        if snap.armed is not False:
            problems.append("vehicle already armed or armed state unknown")
        if snap.ekf_origin_age_s is None or snap.ekf_origin_age_s > c.origin_stale_s:
            problems.append("EKF origin not set (ArduCopter refuses a GUIDED takeoff without it)")
        if snap.position is None or snap.position_age_s is None or snap.position_age_s > c.position_stale_s:
            problems.append("no fresh local position (optical flow / rangefinder / EKF3 sources)")
        if snap.yaw is None:
            problems.append("no local heading yet")
        if not self._map_ok(seen):
            problems.append("no fresh SLAM map (/map) -- is Cartographer running?")
        if not self._slam_ok(seen):
            problems.append("no fresh SLAM pose (TF map -> laser)")
        if c.min_battery_voltage_v is not None:
            if snap.battery_voltage_v is None or snap.battery_age_s is None or snap.battery_age_s > c.battery_stale_s:
                problems.append("no battery voltage reading")
            elif snap.battery_voltage_v < c.min_battery_voltage_v:
                problems.append(f"battery {snap.battery_voltage_v:.2f} V below minimum {c.min_battery_voltage_v:.2f} V")
        return problems

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
        if snap.position is None or snap.position_age_s is None or snap.position_age_s > c.position_stale_s:
            return self._land(now, "local position stale -- landing", FAILED)
        altitude = self._altitude(snap)
        if altitude is not None and altitude > c.takeoff_altitude_m + c.max_altitude_margin_m:
            return self._land(now, f"altitude {altitude:.2f} m above limit -- landing", FAILED)
        if self._battery_below(snap, c.critical_battery_v):
            return self._land(now, f"critical battery ({snap.battery_voltage_v:.2f} V) -- landing where it is", FAILED)
        return None

    def _tick_landing(self, now: float, snap: VehicleSnapshot) -> List[Action]:
        if snap.armed is False and self._link_ok(snap):
            self.state = self._outcome
            self.detail = {
                COMPLETE: "landed at the entry and disarmed -- mission complete",
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
        self._follower.clear()
        return [Action("set_mode", LAND)]

    def _link_ok(self, snap: VehicleSnapshot) -> bool:
        return snap.fcu_connected and snap.state_age_s is not None and snap.state_age_s <= self.config.state_stale_s

    def _slam_ok(self, seen: Perception) -> bool:
        return seen.slam_pose is not None and seen.slam_age_s is not None and seen.slam_age_s <= self.config.slam_stale_s

    def _map_ok(self, seen: Perception) -> bool:
        return seen.map_grid is not None and seen.map_age_s is not None and seen.map_age_s <= self.config.map_stale_s

    def _battery_below(self, snap: VehicleSnapshot, limit: Optional[float]) -> bool:
        return (
            limit is not None
            and snap.battery_voltage_v is not None
            and snap.battery_age_s is not None
            and snap.battery_age_s <= self.config.battery_stale_s
            and snap.battery_voltage_v < limit
        )

    def _request_arm(self, now: float) -> List[Action]:
        self.state = ARMING
        self.detail = "GUIDED confirmed -- arming"
        self._deadline = now + self.config.arm_timeout_s
        return [Action("arm")]

    @staticmethod
    def _pilot_switched(snap: Optional[VehicleSnapshot]) -> bool:
        return snap is not None and snap.mode not in (None, GUIDED, LAND)

    def _pilot_override(self, reason: str) -> List[Action]:
        self.state = PILOT_OVERRIDE
        self.detail = reason
        return []

    def _fail(self, reason: str) -> List[Action]:
        self.state = FAILED
        self.detail = reason
        return []

    def _altitude(self, snap: Optional[VehicleSnapshot]) -> Optional[float]:
        if snap is None or snap.position is None or self._ground is None:
            return None
        return snap.position[2] - self._ground[2]


__all__ = ["Action", "MainConfig", "MainMission", "Perception", "wrap"]

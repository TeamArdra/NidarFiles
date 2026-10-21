"""Unit tests for main_mission_logic.py -- pure decision logic, no ROS, no
vehicle. The full explore/return flight is covered by test_sim2d.py.

Run from D:\\NidarFiles:  python -m pytest missions/main_mission -q
"""
import math

import pytest

from dataclasses import replace

from main_mission_logic import (
    ABORTED, ARMING, COMPLETE, EXPLORING, FAILED, GUIDED, LAND, LANDING, PILOT_OVERRIDE,
    RETURNING, SETTING_GUIDED, TAKING_OFF, Action, MainConfig, MainMission, Perception,
)
from nidar_autonomy.vehicle_snapshot import VehicleSnapshot

CFG = MainConfig(takeoff_altitude_m=1.0, low_battery_v=14.5, critical_battery_v=14.0, explore_budget_s=100.0)


def open_room(size=60):
    """10 cm map of an open 6 m room with walls round it, all seen."""
    data = []
    for r in range(size):
        for c in range(size):
            data.append(100 if r in (0, size - 1) or c in (0, size - 1) else 0)
    return {"info": {"resolution": 0.1, "width": size, "height": size,
                     "origin": {"position": {"x": -3.0, "y": -3.0}}}, "data": data}


ROOM = open_room()


def seen(pose=(0.0, 0.0, 0.0), age=0.1, grid=ROOM, map_age=0.5):
    return Perception(grid, map_age, pose, age)


def snap(mode="STABILIZE", armed=False, pos=(0.0, 0.0, 0.0), connected=True, volts=16.0, yaw=0.0, pos_age=0.1):
    return VehicleSnapshot(connected, 0.2, armed, mode, pos, pos_age, volts, 0.5, 0.5, yaw)


def airborne(cfg=CFG):
    """Drive a mission to EXPLORING at t=3; returns the mission."""
    m = MainMission(cfg)
    assert m.start(0, snap(), seen()) == [Action("set_mode", GUIDED)]
    assert m.tick(0.5, snap(mode=GUIDED), seen()) == [Action("arm")]
    assert m.on_result(1.0, "arm", True, "ok") == [Action("takeoff", 1.0)]
    assert m.on_result(1.1, "takeoff", True, "ok") == []
    assert m.tick(2.0, snap(mode=GUIDED, armed=True, pos=(0, 0, 0.4)), seen()) == []
    assert m.state == TAKING_OFF
    actions = m.tick(3.0, snap(mode=GUIDED, armed=True, pos=(0, 0, 0.95)), seen())
    assert m.state == EXPLORING
    assert actions[0] == Action("set_speed", CFG.cruise_speed_mps)
    assert actions[1].kind == "goto"
    return m


def flying(mode=GUIDED, pos=(0.0, 0.0, 1.0), **kw):
    return snap(mode=mode, armed=True, pos=pos, **kw)


class TestPreflight:
    def test_needs_map_and_slam_pose(self):
        m = MainMission(CFG)
        assert m.start(0, snap(), Perception()) == []
        assert m.state == FAILED
        assert "SLAM map" in m.detail and "SLAM pose" in m.detail

    def test_stale_slam_pose_refused(self):
        m = MainMission(CFG)
        m.start(0, snap(), seen(age=5.0))
        assert m.state == FAILED

    def test_armed_vehicle_refused(self):
        m = MainMission(CFG)
        m.start(0, snap(armed=True), seen())
        assert m.state == FAILED

    def test_low_battery_refused(self):
        m = MainMission(MainConfig(min_battery_voltage_v=15.0))
        m.start(0, snap(volts=14.8), seen())
        assert m.state == FAILED and "battery" in m.detail

    def test_already_guided_goes_straight_to_arm(self):
        m = MainMission(CFG)
        assert m.start(0, snap(mode=GUIDED), seen()) == [Action("arm")]
        assert m.state == ARMING


class TestFlight:
    def test_takeoff_records_entry_and_explores(self):
        m = airborne()
        assert m.status(3.0)["state"] == EXPLORING
        assert m.status(3.0)["distance_to_entry_m"] == 0.0

    def test_goto_targets_cruise_altitude(self):
        m = airborne()
        actions = m.tick(3.2, flying(), seen())
        assert actions and actions[-1].kind == "goto"
        assert actions[-1].value[2] == pytest.approx(1.0)

    def test_open_room_is_mapped_so_returns_and_lands_at_entry(self):
        m = airborne()
        t = 3.0
        for _ in range(3 * 20):  # no frontier in a fully seen room: 3 tries, ~1 s apart
            t += 1.0
            m.tick(t, flying(), seen())
            if m.state == LANDING:
                break
        assert m.state == LANDING
        assert m._outcome == COMPLETE
        m.tick(t + 5, snap(mode=LAND, armed=False), seen())
        assert m.state == COMPLETE

    def test_map_complete_needs_three_tries_a_second_apart(self):
        m = airborne()
        for i in range(15):  # tries at 3.1 and 4.1 s only
            m.tick(3.0 + 0.1 * (i + 1), flying(), seen())
        assert m.state == EXPLORING
        for i in range(10):  # third try at 5.1 s
            m.tick(4.6 + 0.1 * i, flying(), seen())
        assert m.state in (RETURNING, LANDING)

    def test_time_budget_starts_return(self):
        m = airborne()
        m.tick(3.0 + CFG.explore_budget_s + 1, flying(pos=(2.0, 0.0, 1.0)), seen(pose=(2.0, 0.0, 0.0)))
        assert m.state == RETURNING
        assert "time budget" in m.detail

    def test_geofence_starts_return(self):
        m = airborne(MainConfig(geofence_m=1.5))
        m.tick(4.0, flying(pos=(2.0, 0.0, 1.0)), seen(pose=(2.0, 0.0, 0.0)))
        assert m.state == RETURNING
        assert "geofence" in m.detail

    def test_low_battery_returns_critical_lands(self):
        m = airborne()
        m.tick(4.0, flying(pos=(2.0, 0, 1.0), volts=14.3), seen(pose=(2.0, 0.0, 0.0)))
        assert m.state == RETURNING
        assert m.tick(5.0, flying(volts=13.9), seen()) == [Action("set_mode", LAND)]
        assert m.state == LANDING and m._outcome == FAILED

    def test_returning_lands_at_the_entry(self):
        m = airborne()
        m.tick(3.0 + CFG.explore_budget_s + 1, flying(pos=(2.0, 0.0, 1.0)), seen(pose=(2.0, 0.0, 0.0)))
        assert m.state == RETURNING
        actions = m.tick(3.0 + CFG.explore_budget_s + 2, flying(pos=(0.1, 0.1, 1.0)), seen(pose=(0.1, 0.1, 0.0)))
        assert actions == [Action("set_mode", LAND)]
        assert m._outcome == COMPLETE


class TestSafety:
    def test_abort_in_flight_lands_where_it_is(self):
        m = airborne()
        assert m.abort(4.0, flying()) == [Action("set_mode", LAND)]
        assert m.state == LANDING
        m.tick(8.0, snap(mode=LAND, armed=False), seen())
        assert m.state == ABORTED

    def test_abort_while_arming_disarms(self):
        m = MainMission(CFG)
        m.start(0, snap(mode=GUIDED), seen())
        assert m.abort(0.5, snap(mode=GUIDED)) == [Action("disarm")]
        assert m.state == ABORTED

    def test_slam_lost_holds_then_lands(self):
        m = airborne()
        hold = m.tick(4.0, flying(), seen(age=9.0))
        assert m.state == EXPLORING and "holding" in m.detail
        assert hold and hold[0].kind == "goto"
        assert m.tick(4.0 + CFG.slam_lost_land_s + 0.5, flying(), seen(age=9.0)) == [Action("set_mode", LAND)]
        assert m.state == LANDING

    def test_slam_recovers_before_timeout(self):
        m = airborne()
        m.tick(4.0, flying(), seen(age=9.0))
        m.tick(5.0, flying(), seen())
        assert m.state == EXPLORING
        m.tick(4.0 + CFG.slam_lost_land_s + 1, flying(), seen())
        assert m.state == EXPLORING

    def test_pilot_mode_switch_stops_commanding(self):
        m = airborne()
        assert m.tick(4.0, flying(mode="LOITER"), seen()) == []
        assert m.state == PILOT_OVERRIDE

    def test_fcu_land_failsafe_followed(self):
        m = airborne()
        assert m.tick(4.0, flying(mode=LAND), seen()) == []
        assert m.state == LANDING and m._outcome == FAILED

    def test_link_loss_lands(self):
        m = airborne()
        assert m.tick(4.0, flying(connected=False), seen()) == [Action("set_mode", LAND)]

    def test_stale_local_position_lands(self):
        m = airborne()
        assert m.tick(4.0, flying(pos_age=3.0), seen()) == [Action("set_mode", LAND)]

    def test_too_high_lands(self):
        m = airborne()
        assert m.tick(4.0, flying(pos=(0, 0, 1.0 + CFG.max_altitude_margin_m + 0.1)), seen()) == [Action("set_mode", LAND)]

    def test_internal_error_in_flight_lands(self):
        m = airborne()
        assert m.internal_error(4.0, "boom") == [Action("set_mode", LAND)]
        assert m._outcome == FAILED

    def test_takeoff_rejected_lands(self):
        m = MainMission(CFG)
        m.start(0, snap(mode=GUIDED), seen())
        m.on_result(1.0, "arm", True, "ok")
        assert m.on_result(1.1, "takeoff", False, "denied") == [Action("set_mode", LAND)]

    def test_no_slam_pose_at_altitude_lands(self):
        m = MainMission(CFG)
        m.start(0, snap(mode=GUIDED), seen())
        m.on_result(1.0, "arm", True, "ok")
        m.on_result(1.1, "takeoff", True, "ok")
        assert m.tick(3.0, flying(pos=(0, 0, 0.95)), seen(age=9.0)) == [Action("set_mode", LAND)]


class TestFrames:
    def test_jump_between_frames_restarts_the_transform(self):
        """Map frame suddenly rotated 90 deg from local and shifted (e.g. a
        SLAM loop closure): taken at once, not averaged in over seconds."""
        m = airborne()
        # vehicle at local (5, 5) heading +y (pi/2); SLAM says map (0, 0) heading 0
        a = m.tick(3.1, flying(pos=(5.0, 5.0, 1.0), yaw=math.pi / 2), seen(pose=(0.0, 0.0, 0.0)))
        x, y, yaw = m._transform.apply(1.0, 0.0, 0.0)  # 1 m ahead in the map
        assert (x, y) == pytest.approx((5.0, 6.0), abs=0.01)
        assert yaw == pytest.approx(math.pi / 2, abs=0.01)
        assert a and a[-1].kind == "goto"

    def test_small_drift_is_smoothed(self):
        m = airborne(MainConfig(exploration_done_cycles=10_000))  # keep flying
        m.tick(3.1, flying(pos=(0.2, 0.0, 1.0)), seen())  # EKF 0.2 m off the map
        assert m._transform.x == pytest.approx(0.01, abs=0.005)  # 5 % of it per sample
        for t in range(200):
            m.tick(3.2 + t * 0.1, flying(pos=(0.2, 0.0, 1.0)), seen())
        assert m._transform.x == pytest.approx(0.2, abs=0.01)


class TestFreshMap:
    def test_no_free_space_yet_waits_instead_of_finishing(self):
        """A fresh Cartographer map: everything seen still 'not sure' (here
        unknown). Must hold, not call the map complete and land."""
        m = airborne()
        blank = {"info": ROOM["info"], "data": [-1] * len(ROOM["data"])}
        for i in range(20):
            m.tick(3.1 + i, flying(), seen(grid=blank))
        assert m.state == EXPLORING and "waiting for the map" in m.detail
        m.tick(3.1 + CFG.map_ready_timeout_s + 2, flying(), seen(grid=blank))
        assert m.state == LANDING and m._outcome == FAILED

    def test_already_at_the_entry_lands_complete_without_a_path(self):
        m = airborne()
        blank = {"info": ROOM["info"], "data": [-1] * len(ROOM["data"])}
        m.tick(3.0 + CFG.explore_budget_s + 1, flying(), seen(grid=blank))
        assert m.state == RETURNING
        assert m.tick(3.0 + CFG.explore_budget_s + 2, flying(), seen(grid=blank)) == [Action("set_mode", LAND)]
        assert m._outcome == COMPLETE

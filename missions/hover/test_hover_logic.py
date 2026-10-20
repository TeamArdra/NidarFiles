"""Unit tests for hover_logic.py -- pure decision logic, no ROS, no vehicle.
These prove the decisions, not the flight: real behaviour must still be
verified on the vehicle (see D:\\NidarFiles\\README.md).

Run from D:\\NidarFiles:  python -m pytest missions/hover -q
"""
from dataclasses import replace

import pytest

from hover_logic import (
    ABORTED, ARMING, COMPLETE, FAILED, GUIDED, HOVERING, IDLE, LAND, LANDING,
    PILOT_OVERRIDE, SETTING_GUIDED, TAKING_OFF, Action, HoverConfig, HoverMission,
)
from nidar_autonomy.vehicle_snapshot import VehicleSnapshot

CFG = HoverConfig(takeoff_altitude_m=0.5, hold_duration_s=10.0, max_horizontal_drift_m=0.75)
GROUND = (1.0, 2.0, 0.1)


def snap(mode="STABILIZE", armed=False, pos=GROUND, connected=True, state_age=0.2,
         pos_age=0.1, volts=None, volts_age=0.5, origin_age=0.5):
    return VehicleSnapshot(connected, state_age, armed, mode, pos, pos_age, volts, volts_age, origin_age)


def at_height(h, mode=GUIDED, armed=True, dx=0.0):
    return snap(mode=mode, armed=armed, pos=(GROUND[0] + dx, GROUND[1], GROUND[2] + h))


def airborne(mission=None, t=0.0):
    """Drive a mission to HOVERING; returns (mission, time)."""
    m = mission or HoverMission(CFG)
    assert m.start(t, snap()) == [Action("set_mode", GUIDED)]
    assert m.tick(t + 0.5, snap(mode=GUIDED)) == [Action("arm")]
    assert m.on_result(t + 1.0, "arm", True, "ok") == [Action("takeoff", 0.5)]
    assert m.on_result(t + 1.1, "takeoff", True, "ok") == []
    m.tick(t + 2.0, at_height(0.2))
    assert m.state == TAKING_OFF
    m.tick(t + 3.0, at_height(0.45))
    assert m.state == HOVERING
    return m, t + 3.0


class TestNominalFlight:
    def test_full_sequence_guided_arm_takeoff_hold_land_complete(self):
        m, t = airborne()
        assert m.tick(t + 9.0, at_height(0.5)) == []
        assert m.tick(t + 10.0, at_height(0.5)) == [Action("set_mode", LAND)]
        assert m.state == LANDING
        m.tick(t + 12.0, at_height(0.2, mode=LAND))
        assert m.state == LANDING
        m.tick(t + 15.0, at_height(0.0, mode=LAND, armed=False))
        assert m.state == COMPLETE

    def test_already_in_guided_skips_mode_change(self):
        m = HoverMission(CFG)
        assert m.start(0, snap(mode=GUIDED)) == [Action("arm")]
        assert m.state == ARMING

    def test_status_reports_real_execution_and_altitude(self):
        m, t = airborne()
        s = m.status(t + 1.0)
        assert s["execution_mode"] == "real"
        assert s["state"] == HOVERING
        assert s["current_altitude_m"] == pytest.approx(0.45)
        assert s["elapsed_hover_s"] == pytest.approx(1.0)


class TestPreflightNeverArms:
    @pytest.mark.parametrize(
        "bad,why",
        [
            (snap(connected=False), "FCU not connected"),
            (snap(state_age=10.0), "FCU not connected"),
            (snap(armed=True), "already armed"),
            (snap(armed=None), "already armed"),
            (snap(pos=None, pos_age=None), "no fresh local position"),
            (snap(pos_age=5.0), "no fresh local position"),
        ],
    )
    def test_refuses_and_commands_nothing(self, bad, why):
        m = HoverMission(CFG)
        assert m.start(0, bad) == []
        assert m.state == FAILED and why in m.detail

    def test_battery_check_when_configured(self):
        m = HoverMission(replace(CFG, min_battery_voltage_v=14.0))
        assert m.start(0, snap(volts=13.2)) == []
        assert "below minimum" in m.detail
        m2 = HoverMission(replace(CFG, min_battery_voltage_v=14.0))
        assert m2.start(0, snap(volts=None)) == [] and "no battery" in m2.detail
        m3 = HoverMission(replace(CFG, min_battery_voltage_v=14.0))
        assert m3.start(0, snap(volts=15.8)) == [Action("set_mode", GUIDED)]

    def test_start_while_running_is_ignored(self):
        m, t = airborne()
        assert m.start(t, at_height(0.5)) == []
        assert m.state == HOVERING


class TestEarlyFailures:
    def test_guided_not_confirmed_times_out_without_arming(self):
        m = HoverMission(CFG)
        m.start(0, snap())
        assert m.tick(6.0, snap()) == []
        assert m.state == FAILED

    def test_guided_rejected(self):
        m = HoverMission(CFG)
        m.start(0, snap())
        m.on_result(0.2, "set_mode", False, "mode_sent=False")
        assert m.state == FAILED

    def test_arm_failure_disarms_defensively(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        assert m.on_result(1, "arm", False, "prearm: EKF") == [Action("disarm")]
        assert m.state == FAILED and "prearm" in m.detail

    def test_takeoff_rejected_lands(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        m.on_result(1, "arm", True, "ok")
        assert m.on_result(1.2, "takeoff", False, "denied") == [Action("set_mode", LAND)]
        assert m.state == LANDING
        m.tick(3, snap(mode=LAND, armed=False))
        assert m.state == FAILED

    def test_takeoff_never_acknowledged_lands(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        m.on_result(1, "arm", True, "ok")
        assert m.tick(7.0, at_height(0.0)) == [Action("set_mode", LAND)]

    def test_climb_timeout_lands(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        m.on_result(1, "arm", True, "ok")
        m.on_result(1.1, "takeoff", True, "ok")
        assert m.tick(17.0, at_height(0.1)) == [Action("set_mode", LAND)]
        assert "not reached" in m.detail


class TestAbort:
    def test_abort_before_arming_commands_nothing(self):
        m = HoverMission(CFG)
        m.start(0, snap())
        assert m.abort(0.5, snap()) == []
        assert m.state == ABORTED

    def test_abort_while_arming_disarms_on_ground(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        assert m.abort(0.5, snap(mode=GUIDED)) == [Action("disarm")]
        # the in-flight arm request completes afterwards -> disarm again
        assert m.on_result(1.0, "arm", True, "ok") == [Action("disarm")]
        assert m.state == ABORTED

    @pytest.mark.parametrize("phase_height", [0.2, 0.5])
    def test_abort_airborne_lands_never_disarms(self, phase_height):
        m, t = airborne()
        actions = m.abort(t + 1, at_height(phase_height))
        assert actions == [Action("set_mode", LAND)]
        assert Action("disarm") not in actions
        m.tick(t + 5, at_height(0.0, mode=LAND, armed=False))
        assert m.state == ABORTED

    def test_abort_while_landing_marks_aborted(self):
        m, t = airborne()
        m.tick(t + 10, at_height(0.5))
        assert m.abort(t + 11, at_height(0.3, mode=LAND)) == []
        m.tick(t + 14, at_height(0.0, mode=LAND, armed=False))
        assert m.state == ABORTED

    def test_abort_when_idle_does_nothing(self):
        m = HoverMission(CFG)
        assert m.abort(0, snap()) == []
        assert m.state == IDLE


class TestAirborneWatchdogs:
    def test_stale_position_lands(self):
        m, t = airborne()
        assert m.tick(t + 1, replace(at_height(0.5), position_age_s=3.0)) == [Action("set_mode", LAND)]
        assert "stale" in m.detail

    def test_too_high_lands(self):
        m, t = airborne()
        assert m.tick(t + 1, at_height(1.2)) == [Action("set_mode", LAND)]

    def test_drift_lands(self):
        m, t = airborne()
        assert m.tick(t + 1, at_height(0.5, dx=1.0)) == [Action("set_mode", LAND)]

    def test_link_loss_requests_land(self):
        m, t = airborne()
        assert m.tick(t + 1, replace(at_height(0.5), fcu_connected=False)) == [Action("set_mode", LAND)]

    def test_pilot_mode_change_stops_commanding(self):
        m, t = airborne()
        assert m.tick(t + 1, at_height(0.5, mode="ALT_HOLD")) == []
        assert m.state == PILOT_OVERRIDE
        assert m.tick(t + 20, at_height(0.5, mode="ALT_HOLD")) == []
        assert m.abort(t + 21, at_height(0.5, mode="ALT_HOLD")) == []

    def test_fcu_initiated_land_is_followed_not_fought(self):
        m, t = airborne()
        assert m.tick(t + 1, at_height(0.5, mode=LAND)) == []
        assert m.state == LANDING
        assert m.tick(t + 4, at_height(0.3, mode=LAND)) == []
        m.tick(t + 8, at_height(0.0, mode=LAND, armed=False))
        assert m.state == FAILED

    def test_unexpected_disarm_in_air_fails(self):
        m, t = airborne()
        m.tick(t + 1, at_height(0.5, armed=False))
        assert m.state == FAILED


class TestLanding:
    def test_land_request_is_retried_while_still_guided(self):
        m, t = airborne()
        m.tick(t + 10, at_height(0.5))
        assert m.tick(t + 11, at_height(0.5)) == []
        assert m.tick(t + 12.1, at_height(0.5)) == [Action("set_mode", LAND)]

    def test_pilot_takes_over_during_landing(self):
        m, t = airborne()
        m.tick(t + 10, at_height(0.5))
        m.tick(t + 11, at_height(0.4, mode="LOITER"))
        assert m.state == PILOT_OVERRIDE

    def test_pilot_switch_while_arming_stops_the_mission(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        assert m.tick(0.5, snap(mode="STABILIZE")) == []
        assert m.state == PILOT_OVERRIDE
        # the arm request still completes: no takeoff, no disarm under the pilot
        assert m.on_result(1.0, "arm", True, "ok") == []
        assert m.state == PILOT_OVERRIDE

    def test_arm_result_after_pilot_switch_sends_no_takeoff(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        m._snap = snap(mode="STABILIZE")  # switch seen in the snapshot, tick not run yet
        assert m.on_result(0.5, "arm", True, "ok") == []
        assert m.state == PILOT_OVERRIDE

    def test_takeoff_rejected_because_pilot_switched_is_not_landed(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        assert m.on_result(0.5, "arm", True, "ok") == [Action("takeoff", 0.5)]
        m._snap = snap(mode="STABILIZE", armed=True)
        assert m.on_result(0.6, "takeoff", False, "denied") == []
        assert m.state == PILOT_OVERRIDE

    def test_takeoff_rejected_in_guided_still_lands(self):
        m = HoverMission(CFG)
        m.start(0, snap(mode=GUIDED))
        m.on_result(0.5, "arm", True, "ok")
        assert m.on_result(0.6, "takeoff", False, "denied") == [Action("set_mode", LAND)]

    def test_can_start_again_after_completion(self):
        m, t = airborne()
        m.tick(t + 10, at_height(0.5))
        m.tick(t + 13, at_height(0.0, mode=LAND, armed=False))
        assert m.state == COMPLETE
        assert m.start(t + 20, snap(mode=LAND)) == [Action("set_mode", GUIDED)]


# -- internal error (node bug) -------------------------------------------------


def test_internal_error_while_taking_off_lands():
    m = HoverMission(CFG)
    m.start(0.0, snap(mode=GUIDED))
    m.on_result(0.1, "arm", True, "ok")
    m.tick(0.2, at_height(0.0))
    assert m.state == TAKING_OFF
    actions = m.internal_error(0.3, "internal error in takeoff result: ValueError()")
    assert actions == [Action("set_mode", LAND)]
    assert m.state == LANDING and "landing" in m.detail


def test_internal_error_while_arming_disarms():
    m = HoverMission(CFG)
    m.start(0.0, snap(mode=GUIDED))
    assert m.state == ARMING
    assert m.internal_error(0.1, "boom") == [Action("disarm")]
    assert m.state == FAILED


def test_internal_error_when_idle_commands_nothing():
    m = HoverMission(CFG)
    assert m.internal_error(0.0, "boom") == []
    assert m.state == IDLE


def test_internal_error_while_landing_keeps_landing():
    m = HoverMission(CFG)
    m.start(0.0, snap(mode=GUIDED))
    m.on_result(0.1, "arm", True, "ok")
    m.tick(0.2, at_height(0.0))
    m.internal_error(0.3, "first")
    assert m.internal_error(0.4, "again") == []
    assert m.state == LANDING


# -- EKF origin ----------------------------------------------------------------


def test_preflight_refuses_without_ekf_origin_nothing_armed():
    for age in (None, 60.0):
        m = HoverMission(CFG)
        actions = m.start(0.0, snap(origin_age=age))
        assert actions == []
        assert m.state == FAILED and "EKF origin not set" in m.detail


def test_stale_origin_report_in_flight_does_not_crash_or_abort():
    """Regression (2026-10-08 review): the airborne watchdog referenced an
    undefined name when origin reports went stale, which would have raised
    in flight. Origin is a take-off precondition only."""
    m = HoverMission(CFG)
    m.start(0.0, snap(mode=GUIDED))
    m.on_result(0.1, "arm", True, "ok")
    m.tick(0.2, at_height(0.0))
    m.on_result(0.3, "takeoff", True, "ok")
    m.tick(1.0, at_height(0.45))
    assert m.state == HOVERING
    stale = replace(at_height(0.5), ekf_origin_age_s=None)
    assert m.tick(2.0, stale) == []
    assert m.state == HOVERING
    stale = replace(at_height(0.5), ekf_origin_age_s=120.0)
    assert m.tick(3.0, stale) == []
    assert m.state == HOVERING


def test_preflight_passes_with_fresh_ekf_origin():
    m = HoverMission(CFG)
    assert m.start(0.0, snap(origin_age=1.0)) == [Action("set_mode", GUIDED)]

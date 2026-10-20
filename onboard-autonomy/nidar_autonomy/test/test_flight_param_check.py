"""Flight parameter rules, no-RC and RC safety pilot (flight_param_check.py)."""
from nidar_autonomy.flight_param_check import evaluate, names, report

GOOD = {
    "SYSID_MYGCS": 1, "FS_GCS_ENABLE": 5, "FS_GCS_TIMEOUT": 3.0, "FS_EKF_ACTION": 1,
    "BATT_FS_LOW_ACT": 1, "BATT_FS_CRT_ACT": 1, "FS_THR_ENABLE": 0,
    "BATT_ARM_VOLT": 14.7, "BATT_LOW_VOLT": 14.5, "BATT_CRT_VOLT": 14.0, "LOG_DISARMED": 0,
    "ARMING_CHECK": 44470, "FLOW_TYPE": 5, "RNGFND1_TYPE": 10, "RNGFND1_MAX_CM": 800,
    "RNGFND1_ORIENT": 25, "EK3_SRC1_POSXY": 0, "EK3_SRC1_VELXY": 5, "EK3_SRC1_YAW": 1,
    "EK3_SRC_OPTIONS": 0, "COMPASS_USE": 1, "GPS1_TYPE": 0, "GPS2_TYPE": 0,
}


def by_name(results):
    return {r.name: r for r in results}


def test_every_rule_has_a_value_in_the_good_set():
    assert set(names()) == set(GOOD)


def test_good_configuration_is_all_ok():
    results = evaluate(GOOD, mavros_system_id=1)
    assert all(r.ok for r in results)
    assert "ALL OK" in report(results)


def test_float_rounding_from_the_fcu_is_accepted():
    values = dict(GOOD, BATT_ARM_VOLT=14.699999809265137)
    assert by_name(evaluate(values, 1))["BATT_ARM_VOLT"].ok


def test_bench_values_from_today_need_fixing():
    values = dict(GOOD, SYSID_MYGCS=255, FS_GCS_ENABLE=0, BATT_FS_LOW_ACT=0, BATT_FS_CRT_ACT=0,
                  FS_EKF_ACTION=0, BATT_ARM_VOLT=13.0, BATT_LOW_VOLT=13.2, BATT_CRT_VOLT=12.8, LOG_DISARMED=1)
    r = by_name(evaluate(values, 1))
    assert r["SYSID_MYGCS"].fix == 1.0
    assert r["FS_GCS_ENABLE"].fix == 5.0
    assert r["BATT_ARM_VOLT"].fix == 14.7
    assert r["LOG_DISARMED"].fix == 0.0
    assert "can be set with --apply" in report(list(r.values()))


def test_sysid_follows_mavros():
    r = by_name(evaluate(dict(GOOD, SYSID_MYGCS=1), mavros_system_id=255))
    assert not r["SYSID_MYGCS"].ok and r["SYSID_MYGCS"].fix == 255.0


def test_check_only_rules_are_never_auto_fixed():
    values = dict(GOOD, ARMING_CHECK=0, RNGFND1_MAX_CM=8, COMPASS_USE=0)
    r = by_name(evaluate(values, 1))
    for name in ("ARMING_CHECK", "RNGFND1_MAX_CM", "COMPASS_USE"):
        assert not r[name].ok and r[name].fix is None


def test_missing_values_are_unknown_not_fixed():
    values = dict(GOOD)
    del values["FS_GCS_TIMEOUT"]
    r = by_name(evaluate(values, 1))["FS_GCS_TIMEOUT"]
    assert not r.ok and r.value is None and r.fix is None
    assert "UNKNOWN" in report(evaluate(values, 1))


# -- RC safety pilot (--rc) --

GOOD_RC = dict(GOOD, FS_THR_ENABLE=3, FS_OPTIONS=16, FLTMODE1=0, FLTMODE4=2, FLTMODE6=5,
               FLTMODE_CH=5, RC3_MIN=982)


def test_rc_rules_cover_the_good_rc_set():
    assert set(names(rc=True)) == set(GOOD_RC)


def test_good_rc_configuration_is_all_ok():
    results = evaluate(GOOD_RC, mavros_system_id=1, rc=True)
    assert all(r.ok for r in results)
    assert "RC safety pilot" in report(results, rc=True)


def test_no_rc_failsafe_setting_is_fixed_for_rc():
    r = by_name(evaluate(dict(GOOD_RC, FS_THR_ENABLE=0), 1, rc=True))
    assert r["FS_THR_ENABLE"].fix == 3.0


def test_fs_options_keeps_other_bits_when_fixed():
    r = by_name(evaluate(dict(GOOD_RC, FS_OPTIONS=8), 1, rc=True))["FS_OPTIONS"]
    assert not r.ok and r.fix == 24.0
    assert by_name(evaluate(dict(GOOD_RC, FS_OPTIONS=24), 1, rc=True))["FS_OPTIONS"].ok


def test_uncalibrated_rc_is_check_only():
    r = by_name(evaluate(dict(GOOD_RC, RC3_MIN=0), 1, rc=True))["RC3_MIN"]
    assert not r.ok and r.fix is None


def test_no_rc_setup_is_unchanged():
    assert "FS_OPTIONS" not in names()
    assert by_name(evaluate(GOOD, 1))["FS_THR_ENABLE"].ok

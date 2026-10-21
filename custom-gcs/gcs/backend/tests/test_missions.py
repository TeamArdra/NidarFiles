"""Tests for app/missions.py -- the static mission list behind the GCS
Mission dropdown. Pure data/lookup tests, no FastAPI/TestClient needed."""

import pytest

from app.missions import MISSION_REGISTRY, MissionNotFoundError, get_mission


def test_missions_are_hover_motor_test_and_main_missions():
    assert [m.id for m in MISSION_REGISTRY] == ["hover", "motor_test", "main_mission", "main_mission_sim"]
    assert get_mission("hover").name == "Hover"
    assert get_mission("motor_test").name == "Motor Test"
    assert get_mission("main_mission").name == "Main Mission"
    assert get_mission("main_mission_sim").name == "Main Mission (Sim)"


def test_radio_codes_match_the_jetson_table():
    # onboard-autonomy telem_command_codec: MISSION_HOVER == 1, MISSION_MOTOR_TEST == 2,
    # MISSION_MAIN == 3, MISSION_MAIN_SIM == 4
    assert get_mission("hover").radio_code == 1
    assert get_mission("motor_test").radio_code == 2
    assert get_mission("main_mission").radio_code == 3
    assert get_mission("main_mission_sim").radio_code == 4


def test_radio_codes_are_unique_and_nonzero():
    codes = [m.radio_code for m in MISSION_REGISTRY]
    assert len(codes) == len(set(codes))
    assert all(0 < c < 2**24 for c in codes)


def test_unknown_mission_raises():
    with pytest.raises(MissionNotFoundError):
        get_mission("main_nidar")

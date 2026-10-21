"""Unit tests for mission_selection.MissionSelection (pure, no ROS)."""
import pytest

from nidar_autonomy.mission_selection import SELECTION_MAX_AGE_S, MissionSelection


def test_a_fresh_selection_is_handed_to_the_start():
    s = MissionSelection()
    s.update("hover", 10.0)
    assert s.take(10.1) == ("hover", True)


def test_a_selection_is_good_for_one_start_only():
    s = MissionSelection()
    s.update("hover", 10.0)
    s.take(10.1)
    assert s.take(10.2) == (None, False)


def test_no_selection_at_all():
    assert MissionSelection().take(1.0) == (None, False)


def test_an_old_selection_is_not_fresh():
    s = MissionSelection()
    s.update("hover", 0.0)
    assert s.take(SELECTION_MAX_AGE_S + 0.1) == ("hover", False)


def test_a_selection_without_a_mission_id_is_reported_as_none():
    s = MissionSelection()
    s.update(None, 0.0)
    assert s.take(0.1) == (None, True)


def test_a_newer_selection_replaces_an_unused_one():
    s = MissionSelection()
    s.update("hover", 0.0)
    s.update("motor_test", 0.5)
    assert s.take(0.6) == ("motor_test", True)


@pytest.mark.parametrize(
    "selection,expected",
    [
        (("hover", 0.0), (True, None)),  # ours, fresh: run
        (("motor_test", 0.0), (False, None)),  # another mission's START: ignore, not a failure
        ((None, 0.0), (False, None)),  # selection without a mission id: not ours
        (None, (True, "no mission selection received")),
        (("hover", -10.0), (True, "mission selection is stale")),
        (("motor_test", -10.0), (True, "selected mission is 'motor_test', not 'hover'")),
    ],
)
def test_take_for_decides_what_the_hover_node_does_with_a_start(selection, expected):
    s = MissionSelection()
    if selection is not None:
        s.update(*selection)
    assert s.take_for("hover", 0.1) == expected


def _hover_runs(selection: MissionSelection, now: float) -> bool:
    """missions/hover/mission.py runs a START only for (True, None)."""
    return selection.take_for("hover", now) == (True, None)


def _motor_test_runs(selection: MissionSelection, now: float) -> bool:
    """missions/motor_test/mission.py: same rule for its own id."""
    return selection.take_for("motor_test", now) == (True, None)


def test_a_start_that_overtakes_its_selection_runs_no_mission():
    """START hover (selection, then start), then START motor test within the
    5 s window whose "start" reaches the nodes BEFORE its selection (two
    ROS topics, no ordering between them). Without one-shot selections the
    hover node would still hold the fresh "hover" selection from the first
    START and fly the hover instead of the motor test that was asked for."""
    hover, motor_test = MissionSelection(), MissionSelection()
    for s in (hover, motor_test):
        s.update("hover", 0.0)
    assert _hover_runs(hover, 0.01) and not _motor_test_runs(motor_test, 0.01)

    # second START: "start" first ...
    assert not _hover_runs(hover, 2.0)
    assert not _motor_test_runs(motor_test, 2.0)
    # ... then its late selection, which no START uses
    for s in (hover, motor_test):
        s.update("motor_test", 2.01)
    assert not _hover_runs(hover, 30.0) and not _motor_test_runs(motor_test, 30.0)


def test_both_mission_nodes_use_one_shot_selections():
    """Wiring guard: the ROS nodes can't be imported without rclpy, so
    check their source hands every START to MissionSelection.take_for()
    (whose decisions are tested above)."""
    import os

    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "missions")
    for mission in ("hover", "motor_test"):
        with open(os.path.join(root, mission, "mission.py"), encoding="utf-8") as fh:
            source = fh.read()
        assert "MissionSelection()" in source, mission
        assert "self._selection.take_for(MISSION_ID, now)" in source, mission
        assert "_SELECTION_MAX_AGE_S" not in source, mission

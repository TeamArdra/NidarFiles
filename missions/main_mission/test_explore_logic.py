"""Unit tests for explore_logic.py: planning grid, path, follower."""
import math

import pytest

from explore_logic import (
    EntryPose, ExploreConfig, Explorer, FollowConfig, PathFollower, PlanningGrid, plan_path,
)


def grid_from_rows(rows, res=0.1, origin=(0.0, 0.0)):
    """rows[0] is the TOP row as drawn; '#' wall, '.' free, '?' unknown."""
    h, w = len(rows), len(rows[0])
    data = []
    for r in range(h):
        line = rows[h - 1 - r]
        data += [100 if ch == "#" else 0 if ch == "." else -1 for ch in line]
    return {"info": {"resolution": res, "width": w, "height": h,
                     "origin": {"position": {"x": origin[0], "y": origin[1]}}}, "data": data}


CORRIDOR = grid_from_rows(["#" * 30] + ["#" + "." * 28 + "#"] * 10 + ["#" * 30])  # 1 m wide corridor


class TestPlanningGrid:
    def test_inflation_keeps_the_middle_of_a_1m_corridor(self):
        g = PlanningGrid(CORRIDOR, ExploreConfig(inflation_m=0.3))
        mid_row = 6  # y = 0.65 m, middle of the corridor
        assert g.free((15, mid_row))
        assert not g.free((15, 2))  # 0.2 m from the wall line: inside the inflation
        assert not g.free((15, 10))

    def test_too_much_inflation_closes_the_corridor(self):
        g = PlanningGrid(CORRIDOR, ExploreConfig(inflation_m=0.6))
        assert not any(g.free((15, r)) for r in range(g.height))

    def test_unknown_is_blocked(self):
        g = PlanningGrid(grid_from_rows(["....", "????", "...."]), ExploreConfig(inflation_m=0.0))
        assert not g.free((1, 1))
        assert g.free((1, 0))

    def test_behind_the_start_line_is_blocked(self):
        open_ = grid_from_rows(["." * 40] * 40, origin=(-2.0, -2.0))
        entry = EntryPose(0.0, 0.0, math.pi / 2)  # facing +y: "behind" = y < 0
        g = PlanningGrid(open_, ExploreConfig(inflation_m=0.0, behind_start_m=0.5), entry)
        assert g.free(g.cell((0.0, -0.4)))
        assert not g.free(g.cell((0.0, -0.8)))
        assert g.free(g.cell((0.0, 1.5)))

    def test_plan_path_through_corridor(self):
        g = PlanningGrid(CORRIDOR, ExploreConfig(inflation_m=0.3))
        path = plan_path(g, (0.5, 0.6), (2.5, 0.6), 1.0)
        assert path is not None
        assert path[0] == (0.5, 0.6)
        assert all(0.4 <= y <= 0.8 for _, y in path[1:])


class TestExplorer:
    def test_picks_a_frontier_and_blacklists_unreachable_ones(self):
        rows = ["#" * 30] + ["#" + "." * 18 + "?" * 10 + "#"] * 10 + ["#" * 30]
        grid = grid_from_rows(rows)
        ex = Explorer(ExploreConfig(inflation_m=0.3, min_frontier_cells=3))
        choice = ex.choose(grid, (0.5, 0.6), EntryPose(0.5, 0.6, 0.0))
        assert choice is not None
        assert choice.goal[0] > 1.5  # the unknown end of the corridor
        assert choice.path[-1][0] > 1.2

    def test_nothing_to_explore_in_a_closed_room(self):
        ex = Explorer(ExploreConfig(inflation_m=0.3))
        assert ex.choose(CORRIDOR, (0.5, 0.6), EntryPose(0.5, 0.6, 0.0)) is None


class TestFollower:
    def test_carrot_is_lookahead_along_the_path(self):
        f = PathFollower(FollowConfig(lookahead_m=0.6, repulsion_radius_m=0.0))
        f.set_path([(0.0, 0.0), (3.0, 0.0)], 0.0)
        cmd = f.step(0.0, (1.0, 0.0), 0.0, [])
        assert cmd.target == pytest.approx((1.6, 0.0))
        assert not cmd.reached

    def test_carrot_turns_the_corner(self):
        f = PathFollower(FollowConfig(lookahead_m=1.0, repulsion_radius_m=0.0))
        f.set_path([(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)], 0.0)
        cmd = f.step(0.0, (0.5, 0.0), 0.0, [])
        assert cmd.target == pytest.approx((1.0, 0.5))
        assert cmd.yaw == pytest.approx(math.atan2(0.5, 0.5))

    def test_reached_at_the_end(self):
        f = PathFollower(FollowConfig())
        f.set_path([(0.0, 0.0), (1.0, 0.0)], 0.0)
        assert f.step(0.0, (0.9, 0.0), 0.0, []).reached

    def test_wall_pushes_the_setpoint_away(self):
        f = PathFollower(FollowConfig(lookahead_m=0.6))
        f.set_path([(0.0, 0.0), (3.0, 0.0)], 0.0)
        cmd = f.step(0.0, (1.0, 0.0), 0.0, [(1.0, 0.3)])  # wall 0.3 m to the left
        assert cmd.target[1] < 0.0

    def test_stuck_after_no_progress(self):
        f = PathFollower(FollowConfig(stuck_window_s=10.0, stuck_min_move_m=0.4))
        f.set_path([(0.0, 0.0), (5.0, 0.0)], 0.0)
        stuck = False
        for t in range(12):
            f.set_path([(0.0, 0.0), (5.0, 0.0)], float(t))  # replans must not hide it
            stuck = f.step(float(t), (0.1, 0.0), 0.0, []).stuck
        assert stuck

    def test_not_stuck_while_moving(self):
        f = PathFollower(FollowConfig(stuck_window_s=10.0))
        f.set_path([(0.0, 0.0), (10.0, 0.0)], 0.0)
        for t in range(12):
            cmd = f.step(float(t), (0.4 * t, 0.0), 0.0, [])
        assert not cmd.stuck


class TestCartographerValues:
    def test_frontiers_found_on_probability_values(self):
        """Cartographer never writes exactly 0 for free space: a corridor
        of 12s next to unknown must still have a frontier."""
        rows = ["#" * 30] + ["#" + "." * 18 + "?" * 10 + "#"] * 10 + ["#" * 30]
        grid = grid_from_rows(rows)
        grid["data"] = [12 if v == 0 else (70 if v == 100 else v) for v in grid["data"]]
        ex = Explorer(ExploreConfig(inflation_m=0.3, min_frontier_cells=3))
        choice = ex.choose(grid, (0.5, 0.6), EntryPose(0.5, 0.6, 0.0))
        assert choice is not None and choice.goal[0] > 1.5

    def test_reached_frontier_is_not_chosen_again(self):
        rows = ["#" * 30] + ["#" + "." * 18 + "?" * 10 + "#"] * 10 + ["#" * 30]
        grid = grid_from_rows(rows)
        ex = Explorer(ExploreConfig(inflation_m=0.3, min_frontier_cells=3))
        first = ex.choose(grid, (0.5, 0.6), EntryPose(0.5, 0.6, 0.0))
        ex.arrived((first.goal[0] + 0.2, first.goal[1] - 0.1))  # jittered: still the same spot
        assert ex.choose(grid, (0.5, 0.6), EntryPose(0.5, 0.6, 0.0)) is None

"""Closed-loop: the Main Mission logic flies the Gazebo test maze
(sim/worlds/nidar_sim.sdf) in the 2D sim (sim2d.py). Slow-ish (~30 s each)."""
from dataclasses import replace

from main_mission_logic import COMPLETE, MainConfig
from sim2d import Sim2D, load_walls, run


def test_maze_walls_load():
    walls = load_walls()
    assert len(walls) == 18


def test_full_mission_explores_the_maze_returns_and_lands_at_the_entry():
    result = run(MainConfig())
    assert result.state == COMPLETE, "\n".join(result.log)
    assert result.coverage >= 0.95
    assert result.landing_error_m < 0.5
    assert not result.collided
    assert not result.left_arena  # never flew out of the entry gap into the hall
    assert result.sim_time_s < 15 * 60  # inside the 15 min bonus


def test_short_time_budget_still_comes_home():
    result = run(replace(MainConfig(), explore_budget_s=60.0))
    assert result.state == COMPLETE, "\n".join(result.log)
    assert any("time budget" in line for line in result.log)
    assert result.landing_error_m < 0.5
    assert not result.collided


def test_drift_free_and_heading_east_start_also_works():
    """Different start heading (map frame rotated differently) and no EKF
    drift: frames must not matter."""
    sim = Sim2D(load_walls(), start=(0.0, 0.5), start_yaw=1.4, drift_mps=(0.0, 0.0))
    result = run(MainConfig(), sim=sim)
    assert result.state == COMPLETE, "\n".join(result.log)
    assert result.coverage >= 0.9
    assert not result.collided

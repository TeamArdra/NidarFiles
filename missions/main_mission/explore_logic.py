"""Exploration + path following for the Main Mission -- pure, no rclpy.

Everything here works in the SLAM map frame (Cartographer's /map; origin and
heading = where Cartographer started). main_mission_logic.py turns the
setpoints into the FCU's local frame.

Reused, unchanged (onboard-autonomy, already unit-tested):
    frontier_detector.detect_frontiers()   where the known map meets unknown
    exploration_policy.score_candidates()  which frontier next (distance vs
                                           information, visited, hysteresis,
                                           blacklist) + apply_sweep_radius()
    grid_astar_planner.GridAStarPlanner    the path
    occupancy_grid_environment             the grid adapter A* plans on

Added here (ported from TeamArdra/gps_denied path_follower_position.py and
frontier_explorer.py, Rish-736, adapted to these modules):
  * PlanningGrid: the map coarsened to `plan_res_m` and INFLATED by the
    drone's radius + margin -- A* plans a point, so without inflation a path
    may graze a wall. Unknown counts as blocked (never plan through what has
    not been seen). Cells behind the start line are blocked too: the entry is
    on the arena boundary, so whatever the LiDAR sees out of the entry gap is
    the hall, not the arena (see ExploreConfig.behind_start_m).
  * Explorer.choose(): frontier -> nearest reachable free cell -> A* path;
    a frontier that fails is blacklisted (the policy's own blacklist).
  * PathFollower: carrot point `lookahead_m` along the path (bounded step, so
    the drone never gets a far target), a small push away from walls closer
    than `repulsion_radius_m`, face the direction of travel, and a stuck
    check (no progress -> the caller blacklists the goal and replans).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from nidar_autonomy.exploration_policy import (
    apply_sweep_radius,
    record_failure,
    record_visit,
    score_candidates,
)
from nidar_autonomy.frontier_detector import detect_frontiers
from nidar_autonomy.geometry import wrap
from nidar_autonomy.grid_astar_planner import GridAStarPlanner
from nidar_autonomy.occupancy_grid_environment import OccupancyGridEnvironment
from nidar_autonomy.path_planner_interface import PathPlanningError

Point = Tuple[float, float]
Cell = Tuple[int, int]

FREE = 0
BLOCKED = 100


@dataclass(frozen=True)
class ExploreConfig:
    plan_res_m: float = 0.1  # planning cell size (>= the /map resolution)
    inflation_m: float = 0.30  # drone radius incl. guards + margin
    occupied_at: int = 65  # /map value counted as a wall
    free_below: int = 25  # /map value counted as free
    min_frontier_cells: int = 6
    snap_radius_m: float = 1.0  # frontier/start -> nearest free planning cell
    max_goal_attempts: int = 5  # frontiers tried per choose() before giving up
    # Arena limits relative to the entry pose (no arena layout is known):
    behind_start_m: float = 0.5  # block anything further behind the start line
    max_range_m: float = 21.5  # ~15 m x 15 m arena diagonal
    # exploration_policy weights (its own defaults, from gps_denied)
    min_goal_distance_m: float = 0.7
    info_weight: float = 0.4
    hysteresis_bonus: float = 2.0
    visit_cell_m: float = 0.75
    visit_weight: float = 0.6
    visit_radius_m: float = 1.5
    max_failures: int = 3
    sweep_radius_m: float = 5.0


@dataclass(frozen=True)
class FollowConfig:
    lookahead_m: float = 0.6  # carrot distance along the path
    goal_tolerance_m: float = 0.35
    repulsion_radius_m: float = 0.6  # walls closer than this push the setpoint away
    repulsion_gain: float = 0.5
    repulsion_max_m: float = 0.25
    yaw_min_step_m: float = 0.25  # only turn towards the carrot if it is this far
    stuck_window_s: float = 12.0
    stuck_min_move_m: float = 0.4


@dataclass(frozen=True)
class EntryPose:
    """Where the mission started, in the map frame. `yaw` = the direction
    into the arena (the drone is placed facing into it)."""

    x: float
    y: float
    yaw: float

    def ahead(self, x: float, y: float) -> float:
        """Signed distance of (x, y) along the entry heading."""
        return (x - self.x) * math.cos(self.yaw) + (y - self.y) * math.sin(self.yaw)


# --------------------------------------------------------------------------- grid


class PlanningGrid:
    """The /map dict (frontier_detector's shape: info.resolution/width/height/
    origin.position, row-major data, -1 unknown, 0..100 occupancy) reduced
    to a traversable/blocked grid for A*."""

    def __init__(self, grid: dict, cfg: ExploreConfig, entry: Optional[EntryPose] = None) -> None:
        info = grid["info"]
        self.src_res = float(info["resolution"])
        src_w, src_h = int(info["width"]), int(info["height"])
        self.origin = (float(info["origin"]["position"]["x"]), float(info["origin"]["position"]["y"]))
        data = grid["data"]
        factor = max(1, int(round(cfg.plan_res_m / self.src_res)))
        self.res = self.src_res * factor
        self.width = math.ceil(src_w / factor)
        self.height = math.ceil(src_h / factor)
        n = self.width * self.height
        occupied = bytearray(n)
        free = bytearray(n)  # at least one source cell confidently free
        for row in range(src_h):
            crow = row // factor
            base = row * src_w
            for col in range(src_w):
                v = data[base + col]
                i = crow * self.width + col // factor
                if v >= cfg.occupied_at:
                    occupied[i] = 1
                elif 0 <= v < cfg.free_below:
                    free[i] = 1
        self.occupied = occupied
        # traversable = seen free and no wall in it; unknown/uncertain = blocked
        blocked = bytearray(1 if occupied[i] or not free[i] else 0 for i in range(n))
        # inflate walls by the drone's radius
        r = cfg.inflation_m / self.res
        reach = math.ceil(r)
        disk = [(dc, dr) for dr in range(-reach, reach + 1) for dc in range(-reach, reach + 1)
                if math.hypot(dc, dr) <= r + 0.5]
        for i in range(n):
            if not occupied[i]:
                continue
            c0, r0 = i % self.width, i // self.width
            for dc, dr in disk:
                c, rr = c0 + dc, r0 + dr
                if 0 <= c < self.width and 0 <= rr < self.height:
                    blocked[rr * self.width + c] = 1
        # arena limits relative to the entry
        if entry is not None:
            for i in range(n):
                if blocked[i]:
                    continue
                x, y = self.center((i % self.width, i // self.width))
                if entry.ahead(x, y) < -cfg.behind_start_m or math.hypot(x - entry.x, y - entry.y) > cfg.max_range_m:
                    blocked[i] = 1
        self.blocked = blocked

    # -- geometry -----------------------------------------------------------------
    def cell(self, p: Point) -> Cell:
        return (math.floor((p[0] - self.origin[0]) / self.res), math.floor((p[1] - self.origin[1]) / self.res))

    def center(self, c: Cell) -> Point:
        return (self.origin[0] + (c[0] + 0.5) * self.res, self.origin[1] + (c[1] + 0.5) * self.res)

    def inside(self, c: Cell) -> bool:
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def free(self, c: Cell) -> bool:
        return self.inside(c) and not self.blocked[c[1] * self.width + c[0]]

    def nearest_free(self, p: Point, radius_m: float) -> Optional[Point]:
        """Centre of the closest traversable cell within `radius_m` of p
        (breadth-first over cells, any state), or None."""
        start = self.cell(p)
        max_steps = math.ceil(radius_m / self.res)
        if self.free(start):
            return self.center(start)
        seen = {start}
        queue = deque([(start, 0)])
        best = None
        while queue:
            c, d = queue.popleft()
            if d > max_steps:
                break
            if self.free(c):
                q = self.center(c)
                dist = math.hypot(q[0] - p[0], q[1] - p[1])
                if dist <= radius_m and (best is None or dist < best[0]):
                    best = (dist, q)
                continue
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (c[0] + dc, c[1] + dr)
                if n not in seen and self.inside(n):
                    seen.add(n)
                    queue.append((n, d + 1))
        return None if best is None else best[1]

    def as_environment(self) -> OccupancyGridEnvironment:
        return OccupancyGridEnvironment({
            "info": {
                "resolution": self.res, "width": self.width, "height": self.height,
                "origin": {"position": {"x": self.origin[0], "y": self.origin[1]}},
            },
            "data": [BLOCKED if b else FREE for b in self.blocked],
        })

    def wall_points_near(self, p: Point, radius_m: float) -> List[Point]:
        """Centres of occupied (not merely inflated) cells within radius."""
        c0 = self.cell(p)
        reach = math.ceil(radius_m / self.res)
        pts = []
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                c = (c0[0] + dc, c0[1] + dr)
                if self.inside(c) and self.occupied[c[1] * self.width + c[0]]:
                    q = self.center(c)
                    if math.hypot(q[0] - p[0], q[1] - p[1]) <= radius_m:
                        pts.append(q)
        return pts


def plan_path(grid: PlanningGrid, start: Point, goal: Point, snap_m: float) -> Optional[List[Point]]:
    """A* (GridAStarPlanner) between the nearest free cells to start/goal."""
    s = grid.nearest_free(start, snap_m)
    g = grid.nearest_free(goal, snap_m)
    if s is None or g is None:
        return None
    try:
        path = GridAStarPlanner().plan(s, g, grid.as_environment())
    except PathPlanningError:
        return None
    return [start] + path[1:] if math.hypot(s[0] - start[0], s[1] - start[1]) > 1e-6 else path


def path_length(path: Sequence[Point]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


# ------------------------------------------------------------------------ explorer


@dataclass
class Choice:
    goal: Point
    path: List[Point]
    frontier_count: int


@dataclass
class Explorer:
    cfg: ExploreConfig
    visited: Dict[Tuple[int, int], int] = field(default_factory=dict)
    blacklist: Dict[Tuple[float, float], int] = field(default_factory=dict)
    goal: Optional[Point] = None
    frontier_count: int = 0

    def record_pose(self, x: float, y: float) -> None:
        record_visit(self.visited, x, y, self.cfg.visit_cell_m)

    def fail(self, goal: Point) -> None:
        record_failure(self.blacklist, goal[0], goal[1])
        if self.goal == goal:
            self.goal = None

    def choose(self, grid_dict: dict, pose: Point, entry: EntryPose, grid: Optional[PlanningGrid] = None) -> Optional[Choice]:
        """Best reachable frontier and the path to it, or None (= nothing
        reachable left to explore right now)."""
        c = self.cfg
        grid = grid or PlanningGrid(grid_dict, c, entry)
        frontiers = [f for f in detect_frontiers(grid_dict) if f.cell_count >= c.min_frontier_cells]
        # frontiers outside the arena limits are not ours to explore
        frontiers = [
            f for f in frontiers
            if entry.ahead(f.x, f.y) >= -c.behind_start_m and math.hypot(f.x - entry.x, f.y - entry.y) <= c.max_range_m
        ]
        self.frontier_count = len(frontiers)
        scored = score_candidates(
            frontiers, pose=pose, current_goal=self.goal, visited=self.visited, blacklist=self.blacklist,
            min_goal_distance=c.min_goal_distance_m, info_weight=c.info_weight,
            hysteresis_bonus=c.hysteresis_bonus, visit_cell_size=c.visit_cell_m,
            visit_weight=c.visit_weight, visit_radius=c.visit_radius_m, max_failures=c.max_failures,
        )
        # local-first (apply_sweep_radius), the far ones only as fallbacks
        near = apply_sweep_radius(scored, c.sweep_radius_m)
        scored = near + [t for t in scored if t not in near]
        tried = 0
        seen = set()
        for target in scored:
            key = (round(target.x, 2), round(target.y, 2))
            if key in seen:
                continue
            seen.add(key)
            if tried >= c.max_goal_attempts:
                break
            tried += 1
            goal = (target.x, target.y)
            path = plan_path(grid, pose, goal, c.snap_radius_m)
            if path is None or len(path) < 2:
                record_failure(self.blacklist, goal[0], goal[1])
                continue
            self.goal = goal
            return Choice(goal=goal, path=path, frontier_count=len(frontiers))
        self.goal = None
        return None


# ------------------------------------------------------------------------- follower


@dataclass
class FollowCommand:
    target: Point  # map frame setpoint
    yaw: float  # map frame heading
    reached: bool
    stuck: bool
    remaining_m: float


class PathFollower:
    def __init__(self, cfg: FollowConfig) -> None:
        self.cfg = cfg
        self.path: List[Point] = []
        self._index = 0
        self._yaw: Optional[float] = None
        self._progress: deque = deque()

    def set_path(self, path: Sequence[Point], now: float) -> None:
        """New or replanned path. The stuck window is NOT reset here: a
        replan every few seconds must not hide that the drone isn't moving
        (only clear() resets it)."""
        self.path = list(path)
        self._index = 0
        if not self._progress:
            self._progress.append((now, self.path[0]))

    def clear(self) -> None:
        self.path = []
        self._progress.clear()

    def step(self, now: float, pose: Point, yaw: float, walls: Sequence[Point]) -> FollowCommand:
        c = self.cfg
        if self._yaw is None:
            self._yaw = yaw
        if not self.path:
            return FollowCommand(pose, self._yaw, True, False, 0.0)
        # progress along the path: closest segment at or after the current one
        best_i, best_d = self._index, float("inf")
        for i in range(self._index, len(self.path) - 1):
            d = _dist_to_segment(pose, self.path[i], self.path[i + 1])
            if d < best_d - 1e-9:
                best_i, best_d = i, d
            if d > best_d + 2.0:
                break  # well past the nearest part: stop looking further along
        self._index = best_i
        end = self.path[-1]
        to_end = math.hypot(end[0] - pose[0], end[1] - pose[1])
        remaining = to_end if self._index >= len(self.path) - 2 else (
            math.hypot(self.path[self._index + 1][0] - pose[0], self.path[self._index + 1][1] - pose[1])
            + path_length(self.path[self._index + 1:])
        )
        if to_end <= c.goal_tolerance_m:
            return FollowCommand(end, self._yaw, True, False, 0.0)
        carrot = _point_along(self.path, self._index, pose, c.lookahead_m)
        # wall repulsion (low-cost potential field from the mapped walls)
        px = py = 0.0
        for wx, wy in walls:
            dx, dy = pose[0] - wx, pose[1] - wy
            d = math.hypot(dx, dy)
            if 1e-6 < d < c.repulsion_radius_m:
                push = c.repulsion_gain * (c.repulsion_radius_m - d) / c.repulsion_radius_m
                px += push * dx / d
                py += push * dy / d
        mag = math.hypot(px, py)
        if mag > c.repulsion_max_m:
            px, py = px * c.repulsion_max_m / mag, py * c.repulsion_max_m / mag
        target = (carrot[0] + px, carrot[1] + py)
        if math.hypot(carrot[0] - pose[0], carrot[1] - pose[1]) >= c.yaw_min_step_m:
            self._yaw = math.atan2(carrot[1] - pose[1], carrot[0] - pose[0])
        # stuck: moved less than stuck_min_move_m over the last stuck_window_s
        self._progress.append((now, pose))
        while self._progress and now - self._progress[0][0] > c.stuck_window_s:
            self._progress.popleft()
        oldest_t, oldest_p = self._progress[0]
        stuck = (
            now - oldest_t >= c.stuck_window_s * 0.95
            and math.hypot(pose[0] - oldest_p[0], pose[1] - oldest_p[1]) < c.stuck_min_move_m
        )
        return FollowCommand(target, self._yaw, False, stuck, remaining)


def _dist_to_segment(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg = dx * dx + dy * dy
    t = 0.0 if seg == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _point_along(path: Sequence[Point], index: int, pose: Point, distance: float) -> Point:
    """The point `distance` metres along the path from pose's projection on
    segment `index`."""
    ax, ay = path[index]
    bx, by = path[index + 1]
    dx, dy = bx - ax, by - ay
    seg = math.hypot(dx, dy)
    t = 0.0 if seg == 0 else max(0.0, min(1.0, ((pose[0] - ax) * dx + (pose[1] - ay) * dy) / (seg * seg)))
    cur = (ax + t * dx, ay + t * dy)
    left = distance
    i = index
    while True:
        nxt = path[i + 1]
        d = math.hypot(nxt[0] - cur[0], nxt[1] - cur[1])
        if d >= left:
            f = left / d if d > 0 else 0.0
            return (cur[0] + f * (nxt[0] - cur[0]), cur[1] + f * (nxt[1] - cur[1]))
        left -= d
        cur = nxt
        i += 1
        if i >= len(path) - 1:
            return path[-1]


__all__ = [
    "Choice", "EntryPose", "ExploreConfig", "Explorer", "FollowCommand", "FollowConfig",
    "PathFollower", "PlanningGrid", "path_length", "plan_path", "wrap",
]

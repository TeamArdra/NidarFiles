"""2D closed-loop test sim for the Main Mission -- pure Python, no ROS.

Flies main_mission_logic.MainMission through the Gazebo test maze
(sim/worlds/nidar_sim.sdf, its wall boxes read straight from the SDF) with:
  - a 2D LiDAR (ray cast) building the map the way Cartographer would,
    published in a map frame anchored at the start pose (Cartographer's);
  - the FCU's local frame drifting slowly away from the truth (optical
    flow), so the map -> local transform is really exercised;
  - a vehicle that flies to the last GUIDED target at a limited speed,
    climbs on TAKEOFF and descends + disarms on LAND;
  - a collision check against the true walls.

It proves the DECISIONS (explore, return, land, failsafes) on the real
maze layout, quickly and on any laptop. It does not prove flight dynamics,
Cartographer or MAVROS -- that is what Main Mission (Sim) in Gazebo is for.

    python missions/main_mission/sim2d.py          # run once, print the result + map
"""

from __future__ import annotations

import math
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
try:
    import nidar_autonomy  # noqa: F401
except ImportError:
    sys.path.insert(0, os.path.join(_HERE, "..", "..", "onboard-autonomy", "nidar_autonomy"))

from main_mission_logic import (  # noqa: E402
    COMPLETE, GUIDED, LAND, TERMINAL_STATES, Action, MainConfig, MainMission, Perception,
)
from nidar_autonomy.vehicle_snapshot import VehicleSnapshot  # noqa: E402

WORLD_SDF = os.path.join(_HERE, "..", "..", "sim", "worlds", "nidar_sim.sdf")
Rect = Tuple[float, float, float, float]


def load_walls(path: str = WORLD_SDF) -> List[Rect]:
    """Axis-aligned wall boxes (x_min, y_min, x_max, y_max) from the SDF's
    <collision> boxes."""
    text = open(path, encoding="utf-8").read()
    walls = []
    for m in re.finditer(
        r"<collision name=\"[^\"]+\">\s*<pose>([-\d.]+) ([-\d.]+) [-\d.]+ [^<]*</pose>\s*"
        r"<geometry><box><size>([-\d.]+) ([-\d.]+) [-\d.]+</size>",
        text,
    ):
        x, y, sx, sy = map(float, m.groups())
        walls.append((x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2))
    return walls


@dataclass
class SimResult:
    state: str
    detail: str
    sim_time_s: float
    coverage: float  # fraction of the arena's free cells mapped
    landing_error_m: Optional[float]  # landing point to the take-off point
    min_clearance_m: float  # closest the drone's centre came to a wall
    collided: bool
    left_arena: bool
    log: List[str] = field(default_factory=list)


class Sim2D:
    RES = 0.1
    BOUNDS = (-8.0, -4.0, 8.0, 14.0)  # world x_min, y_min, x_max, y_max -- includes a "hall" south of the entry

    def __init__(self, walls: List[Rect], start=(0.0, 0.0), start_yaw=math.pi / 2,
                 drift_mps=(0.004, -0.003), speed_mps=0.5, lidar_range_m=6.0,
                 body_radius_m=0.2) -> None:
        self.walls = walls
        self.start = start
        self.start_yaw = start_yaw
        self.drift = drift_mps
        self.speed = speed_mps
        self.lidar_range = lidar_range_m
        self.body_radius = body_radius_m
        x0, y0, x1, y1 = self.BOUNDS
        self.w = round((x1 - x0) / self.RES)
        self.h = round((y1 - y0) / self.RES)
        self.occ = bytearray(self.w * self.h)
        for (ax, ay, bx, by) in walls:
            for r in range(max(0, self._row(ay)), min(self.h, self._row(by - 1e-9) + 1)):
                for c in range(max(0, self._col(ax)), min(self.w, self._col(bx - 1e-9) + 1)):
                    self.occ[r * self.w + c] = 1
        self.known = bytearray(b"\xff" * (self.w * self.h))  # 255 unknown, 0 free, 100 wall
        # vehicle truth
        self.x, self.y, self.z, self.yaw = start[0], start[1], 0.0, start_yaw
        self.armed = False
        self.mode = "STABILIZE"
        self.target: Optional[Tuple[float, float, float, float]] = None  # world frame
        self.max_speed = speed_mps
        self.t = 0.0
        self.min_clearance = math.inf
        self.left_arena = False
        self._map: Optional[dict] = None
        self._map_at = -math.inf
        self._scan_at = -math.inf

    # -- frames -------------------------------------------------------------------
    def _col(self, x: float) -> int:
        return math.floor((x - self.BOUNDS[0]) / self.RES)

    def _row(self, y: float) -> int:
        return math.floor((y - self.BOUNDS[1]) / self.RES)

    def _wall(self, x: float, y: float) -> bool:
        c, r = self._col(x), self._row(y)
        return not (0 <= c < self.w and 0 <= r < self.h) or bool(self.occ[r * self.w + c])

    def _offset(self) -> Tuple[float, float]:
        return (self.drift[0] * self.t, self.drift[1] * self.t)

    def world_to_map(self, x, y, yaw):
        dx, dy = x - self.start[0], y - self.start[1]
        c, s = math.cos(-self.start_yaw), math.sin(-self.start_yaw)
        return (c * dx - s * dy, s * dx + c * dy, yaw - self.start_yaw)

    def map_to_world(self, mx, my):
        c, s = math.cos(self.start_yaw), math.sin(self.start_yaw)
        return (self.start[0] + c * mx - s * my, self.start[1] + s * mx + c * my)

    # -- sensors --------------------------------------------------------------------
    def _scan(self) -> None:
        step = self.RES
        for k in range(180):
            a = self.yaw + k * 2 * math.pi / 180
            ca, sa = math.cos(a), math.sin(a)
            d = 0.0
            while d < self.lidar_range:
                px, py = self.x + d * ca, self.y + d * sa
                c, r = self._col(px), self._row(py)
                if not (0 <= c < self.w and 0 <= r < self.h):
                    break
                i = r * self.w + c
                if self.occ[i]:
                    self.known[i] = 100
                    break
                if self.known[i] != 100:
                    self.known[i] = 0
                d += step

    def _build_map(self) -> dict:
        """The known grid resampled into the map frame (Cartographer's /map)."""
        corners = [self.world_to_map(x, y, 0)[:2] for x in (self.BOUNDS[0], self.BOUNDS[2]) for y in (self.BOUNDS[1], self.BOUNDS[3])]
        mx0 = min(p[0] for p in corners)
        my0 = min(p[1] for p in corners)
        mw = round((max(p[0] for p in corners) - mx0) / self.RES)
        mh = round((max(p[1] for p in corners) - my0) / self.RES)
        data = [-1] * (mw * mh)
        for r in range(mh):
            for c in range(mw):
                wx, wy = self.map_to_world(mx0 + (c + 0.5) * self.RES, my0 + (r + 0.5) * self.RES)
                wc, wr = self._col(wx), self._row(wy)
                if 0 <= wc < self.w and 0 <= wr < self.h:
                    v = self.known[wr * self.w + wc]
                    if v != 255:
                        data[r * mw + c] = v
        return {"info": {"resolution": self.RES, "width": mw, "height": mh,
                         "origin": {"position": {"x": mx0, "y": my0}}}, "data": data}

    def snapshot(self) -> VehicleSnapshot:
        ox, oy = self._offset()
        return VehicleSnapshot(
            fcu_connected=True, state_age_s=0.2, armed=self.armed, mode=self.mode,
            position=(self.x + ox, self.y + oy, self.z), position_age_s=0.05,
            battery_voltage_v=16.0, battery_age_s=0.5, ekf_origin_age_s=0.5, yaw=self.yaw,
        )

    def perception(self) -> Perception:
        if self.t - self._scan_at >= 0.5:
            self._scan()
            self._scan_at = self.t
        if self._map is None or self.t - self._map_at >= 1.0:
            self._map = self._build_map()
            self._map_at = self.t
        return Perception(self._map, self.t - self._map_at, self.world_to_map(self.x, self.y, self.yaw), 0.05)

    # -- vehicle --------------------------------------------------------------------
    def execute(self, action: Action) -> Tuple[bool, str]:
        if action.kind == "set_mode":
            self.mode = action.value
            if action.value == LAND:
                self.target = None
            return True, f"mode {action.value}"
        if action.kind == "arm":
            if self.mode != GUIDED:
                return False, "not in GUIDED"
            self.armed = True
            return True, "armed"
        if action.kind == "disarm":
            self.armed = False
            return True, "disarmed"
        if action.kind == "takeoff":
            self.target = (self.x, self.y, float(action.value), self.yaw)
            return True, "takeoff"
        if action.kind == "set_speed":
            self.max_speed = min(self.speed, float(action.value))
            return True, "speed"
        if action.kind == "goto":
            lx, ly, lz, lyaw = action.value
            ox, oy = self._offset()
            if self.mode == GUIDED and self.armed:
                self.target = (lx - ox, ly - oy, lz, lyaw)
            return True, "goto"
        return False, f"unknown action {action.kind}"

    def step(self, dt: float) -> None:
        self.t += dt
        if not self.armed:
            return
        if self.mode == LAND:
            self.z = max(0.0, self.z - 0.5 * dt)
            if self.z <= 0.0:
                self.armed = False  # ArduCopter disarms after touchdown
            return
        if self.target is not None:
            tx, ty, tz, tyaw = self.target
            dx, dy = tx - self.x, ty - self.y
            d = math.hypot(dx, dy)
            move = min(d, self.max_speed * dt)
            if d > 1e-9:
                self.x += dx / d * move
                self.y += dy / d * move
            self.z += max(-0.5 * dt, min(0.5 * dt, tz - self.z))
            dyaw = math.atan2(math.sin(tyaw - self.yaw), math.cos(tyaw - self.yaw))
            self.yaw += max(-1.0 * dt, min(1.0 * dt, dyaw))
        clearance = min(_rect_distance(self.x, self.y, w) for w in self.walls)
        self.min_clearance = min(self.min_clearance, clearance)
        if self.y < -1.2 and self.z > 0.2:
            self.left_arena = True

    def coverage(self) -> float:
        """Mapped fraction of the free cells inside the arena walls."""
        free = mapped = 0
        for r in range(self._row(-0.9), self._row(12.9) + 1):
            for c in range(self._col(-6.9), self._col(6.9) + 1):
                i = r * self.w + c
                if not self.occ[i]:
                    free += 1
                    mapped += self.known[i] == 0
        return mapped / free if free else 0.0

    def ascii_map(self, path: Optional[List[Tuple[float, float]]] = None) -> str:
        pts = {(self._col(x), self._row(y)) for x, y in (path or [])}
        rows = []
        for r in range(self.h - 1, -1, -2):
            line = []
            for c in range(0, self.w, 2):
                i = r * self.w + c
                if (c, r) in pts:
                    line.append("*")
                elif self.occ[i]:
                    line.append("#" if self.known[i] == 100 else "+")
                else:
                    line.append(" " if self.known[i] == 0 else ".")
            rows.append("".join(line))
        return "\n".join(rows)


def _rect_distance(x: float, y: float, rect: Rect) -> float:
    ax, ay, bx, by = rect
    dx = max(ax - x, 0.0, x - bx)
    dy = max(ay - y, 0.0, y - by)
    return math.hypot(dx, dy)


def run(config: Optional[MainConfig] = None, sim: Optional[Sim2D] = None, dt: float = 0.2,
        max_time_s: float = 1500.0, track: Optional[List[Tuple[float, float]]] = None) -> SimResult:
    config = config or MainConfig()
    sim = sim or Sim2D(load_walls())
    mission = MainMission(config, "main_mission_sim")
    log: List[str] = []
    last = None

    def do(actions):
        for a in actions:
            ok, detail = sim.execute(a)
            if a.kind != "goto":
                do(mission.on_result(sim.t, a.kind, ok, detail))

    do(mission.start(sim.t, sim.snapshot(), sim.perception()))
    while sim.t < max_time_s:
        if (mission.state, mission.detail) != last:
            last = (mission.state, mission.detail)
            log.append(f"{sim.t:7.1f}s {mission.state}: {mission.detail}")
        if mission.state in TERMINAL_STATES:
            break
        do(mission.tick(sim.t, sim.snapshot(), sim.perception()))
        sim.step(dt)
        if track is not None and sim.armed:
            track.append((sim.x, sim.y))
    landing = math.hypot(sim.x - sim.start[0], sim.y - sim.start[1]) if mission.state == COMPLETE else None
    return SimResult(
        state=mission.state, detail=mission.detail, sim_time_s=round(sim.t, 1),
        coverage=sim.coverage(), landing_error_m=landing, min_clearance_m=sim.min_clearance,
        collided=sim.min_clearance < sim.body_radius, left_arena=sim.left_arena, log=log,
    )


if __name__ == "__main__":
    track: List[Tuple[float, float]] = []
    sim = Sim2D(load_walls())
    result = run(sim=sim, track=track)
    print("\n".join(result.log))
    print(sim.ascii_map(track))
    print(f"state={result.state} time={result.sim_time_s}s coverage={result.coverage:.0%} "
          f"landing_error={result.landing_error_m} min_clearance={result.min_clearance_m:.2f} m "
          f"collided={result.collided} left_arena={result.left_arena}")

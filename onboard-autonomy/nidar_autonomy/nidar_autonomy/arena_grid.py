"""The SLAM map as the organiser's 1 m arena grid -- pure Python (unit-tested), no rclpy.

Scoring (Rulebook §9, AirMouse Phase 4D) checks the map per 1 m x 1 m grid
cell: walls, openings and accessible directions around each cell. Survivor
tags are also judged by 1 m grid box. Cartographer's /map is a fine grid in
its own frame (origin and heading = where the drone started), so this module:

1. Alignment: finds the arena's wall directions (the maze is a modular grid,
   so walls run along two perpendicular axes) and where the 1 m grid lines
   fall, from the occupied cells alone. No operator input.
2. Edges: classifies every cell edge as WALL, OPEN or UNKNOWN by sampling
   the fine map along it.
3. Grid frame: cell (0, 0) is the cell the drone started in (the entry
   point); +col along the arena axis nearest the drone's initial heading,
   +row 90 degrees counter-clockwise from that. Mapping these indices to the
   organiser's labels (A1, ...) is left to the GCS: the convention isn't
   published yet.

Edge states use the same 2-bit values as map_grid (UNKNOWN 0, FREE 1 = open,
OCCUPIED 2 = wall), so map_grid.pack() packs them unchanged.

Assumes thin walls standing on the 1 m grid lines (corridors 1 m, rooms 2 m,
Annexure 2 §3).

Over the radio: MAVLink TUNNEL type ARENA_GRID, the grid split into parts of
<= 128 bytes (a 15 x 15 grid is 2 parts). custom-gcs/gcs/backend/app/
radio_protocol.py mirrors the format.

ARENA_GRID payload:
    <B gen> <B part> <B nparts> <h theta_cdeg> <H offset_u_mm> <H offset_v_mm>
    <h entry_u> <h entry_v> <b col0> <b row0> <B cols> <B rows>
    <edges, 2 bits each, 4 per byte, first in the low bits>
Edge stream: v_edges row by row (rows x (cols + 1)), then h_edges row by row
((rows + 1) x cols); part k carries edges k * EDGES_PER_PART onwards. gen
changes whenever the grid does, so the GCS never mixes parts of two grids.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .map_grid import FREE_MAX, OCCUPIED_MIN, TUNNEL_PAYLOAD_MAX, pack

UNKNOWN, OPEN, WALL = 0, 1, 2

CELL_M = 1.0
MIN_WALL_POINTS = 40  # fewer occupied fine cells than this: no alignment yet
MAX_ALIGN_POINTS = 4000  # occupied cells used for alignment (evenly subsampled)
COARSE_STEP_DEG = 1.0
FINE_STEP_DEG = 0.1
EDGE_SAMPLES = 10  # samples along each edge, over its middle 70 %
EDGE_MARGIN = 0.15  # fraction of the edge skipped at each end (corners)
EDGE_BAND_M = 0.15  # half-width of the band searched across the grid line
WALL_MIN_FRACTION = 0.4
OPEN_MIN_FRACTION = 0.6
OPEN_MAX_WALL_FRACTION = 0.2
MAX_CELLS = 20  # per axis; the arena is at most 15 x 15
NEAR_WINDOW_DEG = 2.0  # rotation search around the previous alignment
LOCK_MIN_POINTS = 300  # ...used only if that one saw at least this much wall

ARENA_GRID_TYPE = 0x8003
_ARENA_HEADER = struct.Struct("<BBBhHHhhbbBB")
EDGES_PER_PART = (TUNNEL_PAYLOAD_MAX - _ARENA_HEADER.size) * 4


@dataclass(frozen=True)
class OccupancyMap:
    """A nav_msgs/OccupancyGrid's fields (row-major, row 0 at origin_y,
    origin assumed unrotated, as Cartographer publishes it)."""

    data: Sequence[int]
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def value_at(self, x: float, y: float) -> int:
        c = math.floor((x - self.origin_x) / self.resolution)
        r = math.floor((y - self.origin_y) / self.resolution)
        if 0 <= c < self.width and 0 <= r < self.height:
            return self.data[r * self.width + c]
        return -1


@dataclass(frozen=True)
class Alignment:
    """Arena frame in the map frame: u = x cos(theta) + y sin(theta),
    v = -x sin(theta) + y cos(theta); grid lines at u = offset_u + k,
    v = offset_v + k. theta is in [-45, 45) degrees, offsets in [0, 1)."""

    theta_rad: float
    offset_u: float
    offset_v: float
    support: int = 0  # occupied cells it was estimated from

    def to_arena(self, x: float, y: float) -> Tuple[float, float]:
        c, s = math.cos(self.theta_rad), math.sin(self.theta_rad)
        return x * c + y * s, -x * s + y * c

    def to_map(self, u: float, v: float) -> Tuple[float, float]:
        c, s = math.cos(self.theta_rad), math.sin(self.theta_rad)
        return u * c - v * s, u * s + v * c


@dataclass(frozen=True)
class ArenaGrid:
    """Edge states of the explored part of the arena.

    Cells are indexed (col, row) relative to the entry cell. The grid covers
    cols col0 .. col0 + cols - 1 and rows row0 .. row0 + rows - 1.
    v_edges[r][k]: the edge on the west side of column col0 + k, row row0 + r
                   (k = cols is the east side of the last column).
    h_edges[k][c]: the edge on the south side of row row0 + k, column col0 + c
                   (k = rows is the north side of the last row)."""

    alignment: Alignment
    entry_u: int  # grid-line index (floor) of the entry cell's west side
    entry_v: int
    col0: int
    row0: int
    cols: int
    rows: int
    v_edges: List[List[int]]
    h_edges: List[List[int]]

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        """Map-frame point -> (col, row) of its cell, relative to the entry cell."""
        return cell_index(self.alignment, self.entry_u, self.entry_v, x, y)

    def cell_walls(self, col: int, row: int) -> Optional[Tuple[int, int, int, int]]:
        """(north, east, south, west) edge states of a cell, or None outside the grid."""
        c, r = col - self.col0, row - self.row0
        if not (0 <= c < self.cols and 0 <= r < self.rows):
            return None
        return self.h_edges[r + 1][c], self.v_edges[r][c + 1], self.h_edges[r][c], self.v_edges[r][c]


def cell_index(al: Alignment, entry_u: int, entry_v: int, x: float, y: float) -> Tuple[int, int]:
    u, v = al.to_arena(x, y)
    return math.floor(u - al.offset_u) - entry_u, math.floor(v - al.offset_v) - entry_v


def _occupied_points(m: OccupancyMap) -> List[Tuple[float, float]]:
    pts = []
    res = m.resolution
    for r in range(m.height):
        base = r * m.width
        y = m.origin_y + (r + 0.5) * res
        for c in range(m.width):
            if m.data[base + c] >= OCCUPIED_MIN:
                pts.append((m.origin_x + (c + 0.5) * res, y))
    return pts


def _sharpness(pts: Sequence[Tuple[float, float]], theta: float, bin_m: float) -> int:
    """How strongly the points pile up on lines along the two axes at theta:
    sum of squared histogram counts of both projections."""
    c, s = math.cos(theta), math.sin(theta)
    hu: dict = {}
    hv: dict = {}
    for x, y in pts:
        ku = math.floor((x * c + y * s) / bin_m)
        kv = math.floor((-x * s + y * c) / bin_m)
        hu[ku] = hu.get(ku, 0) + 1
        hv[kv] = hv.get(kv, 0) + 1
    return sum(n * n for n in hu.values()) + sum(n * n for n in hv.values())


def _phase(values: Sequence[float]) -> float:
    """Circular mean of values modulo CELL_M, in [0, CELL_M). Points on walls
    across this axis share one phase; points along walls on this axis are
    spread over whole cells and cancel out."""
    sx = sy = 0.0
    for val in values:
        a = 2.0 * math.pi * val / CELL_M
        sx += math.cos(a)
        sy += math.sin(a)
    return (math.atan2(sy, sx) / (2.0 * math.pi) * CELL_M) % CELL_M


def estimate_alignment(m: OccupancyMap, near: Optional[Alignment] = None) -> Optional[Alignment]:
    """Arena axes and grid-line offsets from the occupied cells, or None if
    too little wall has been mapped yet. near: the previous alignment; if it
    was estimated from at least LOCK_MIN_POINTS wall cells, the rotation is
    only searched within NEAR_WINDOW_DEG of it (much less work, and the grid
    can't jump to a different solution mid-flight)."""
    pts = _occupied_points(m)
    if len(pts) < MIN_WALL_POINTS:
        return None
    if len(pts) > MAX_ALIGN_POINTS:
        step = len(pts) / MAX_ALIGN_POINTS
        pts = [pts[int(i * step)] for i in range(MAX_ALIGN_POINTS)]
    bin_m = max(m.resolution, 0.05)

    def best(degs):
        return max(degs, key=lambda d: _sharpness(pts, math.radians(d), bin_m))

    support = len(pts)
    if near is None or near.support < LOCK_MIN_POINTS:
        coarse = best([-45.0 + i * COARSE_STEP_DEG for i in range(int(90 / COARSE_STEP_DEG))])
        n = int(COARSE_STEP_DEG / FINE_STEP_DEG)
    else:
        coarse = math.degrees(near.theta_rad)
        n = int(NEAR_WINDOW_DEG / FINE_STEP_DEG)
    deg = best([coarse + i * FINE_STEP_DEG for i in range(-n, n + 1)])
    deg = (deg + 45.0) % 90.0 - 45.0
    theta = math.radians(deg)

    c, s = math.cos(theta), math.sin(theta)
    us = [x * c + y * s for x, y in pts]
    vs = [-x * s + y * c for x, y in pts]
    return Alignment(theta, _phase(us), _phase(vs), support)


def _edge_state(m: OccupancyMap, al: Alignment, along_u: bool, fixed: float, start: float) -> int:
    """State of the grid-line segment fixed = const, from start to start + 1.
    along_u: the segment runs along u (a horizontal edge, v = fixed)."""
    res = m.resolution
    nband = max(1, int(round(EDGE_BAND_M / res)))
    wall = free = 0
    span = 1.0 - 2 * EDGE_MARGIN
    for i in range(EDGE_SAMPLES):
        t = start + EDGE_MARGIN + span * (i + 0.5) / EDGE_SAMPLES
        hit = False
        free_before = free_after = False
        for k in range(-nband, nband + 1):
            d = fixed + k * res
            x, y = al.to_map(t, d) if along_u else al.to_map(d, t)
            val = m.value_at(x, y)
            if val >= OCCUPIED_MIN:
                hit = True
                break
            if 0 <= val <= FREE_MAX:
                if k < 0:
                    free_before = True
                elif k > 0:
                    free_after = True
        if hit:
            wall += 1
        elif free_before and free_after:  # seen free on both sides: can pass here
            free += 1
    if wall >= WALL_MIN_FRACTION * EDGE_SAMPLES:
        return WALL
    if free >= OPEN_MIN_FRACTION * EDGE_SAMPLES and wall <= OPEN_MAX_WALL_FRACTION * EDGE_SAMPLES:
        return OPEN
    return UNKNOWN


def _known_bounds(m: OccupancyMap, al: Alignment) -> Optional[Tuple[int, int, int, int]]:
    """Grid-line index range (u0, u1, v0, v1) of cells holding any known map cell."""
    res = m.resolution
    lo_u = lo_v = math.inf
    hi_u = hi_v = -math.inf
    for r in range(m.height):
        base = r * m.width
        y = m.origin_y + (r + 0.5) * res
        for c in range(m.width):
            if m.data[base + c] < 0:
                continue
            u, v = al.to_arena(m.origin_x + (c + 0.5) * res, y)
            lo_u, hi_u = min(lo_u, u), max(hi_u, u)
            lo_v, hi_v = min(lo_v, v), max(hi_v, v)
    if lo_u is math.inf:
        return None
    return (
        math.floor(lo_u - al.offset_u),
        math.floor(hi_u - al.offset_u),
        math.floor(lo_v - al.offset_v),
        math.floor(hi_v - al.offset_v),
    )


def build_grid(
    m: OccupancyMap, al: Alignment, start_x: float = 0.0, start_y: float = 0.0
) -> Optional[ArenaGrid]:
    """Edge states for every cell touching the known map. start_x/y: where the
    drone took off in the map frame (Cartographer's origin by default); its
    cell is (0, 0). Clipped to MAX_CELLS per axis around the entry cell."""
    bounds = _known_bounds(m, al)
    if bounds is None:
        return None
    su, sv = al.to_arena(start_x, start_y)
    eu = math.floor(su - al.offset_u)
    ev = math.floor(sv - al.offset_v)
    u0, u1, v0, v1 = bounds
    u0, u1 = max(u0, eu - MAX_CELLS + 1), min(u1, eu + MAX_CELLS - 1)
    v0, v1 = max(v0, ev - MAX_CELLS + 1), min(v1, ev + MAX_CELLS - 1)
    if u1 - u0 + 1 > MAX_CELLS:  # keep the side nearer the entry
        if eu - u0 < u1 - eu:
            u1 = u0 + MAX_CELLS - 1
        else:
            u0 = u1 - MAX_CELLS + 1
    if v1 - v0 + 1 > MAX_CELLS:
        if ev - v0 < v1 - ev:
            v1 = v0 + MAX_CELLS - 1
        else:
            v0 = v1 - MAX_CELLS + 1
    cols, rows = u1 - u0 + 1, v1 - v0 + 1

    v_edges = [
        [_edge_state(m, al, False, al.offset_u + (u0 + k), al.offset_v + (v0 + r)) for k in range(cols + 1)]
        for r in range(rows)
    ]
    h_edges = [
        [_edge_state(m, al, True, al.offset_v + (v0 + k), al.offset_u + (u0 + c)) for c in range(cols)]
        for k in range(rows + 1)
    ]
    return ArenaGrid(al, eu, ev, u0 - eu, v0 - ev, cols, rows, v_edges, h_edges)


def edge_stream(grid: ArenaGrid) -> List[int]:
    return [e for row in grid.v_edges for e in row] + [e for row in grid.h_edges for e in row]


def encode_parts(grid: ArenaGrid, gen: int) -> List[bytes]:
    """ARENA_GRID TUNNEL payloads for the whole grid (see the module docstring)."""
    al = grid.alignment
    edges = edge_stream(grid)
    nparts = max(1, (len(edges) + EDGES_PER_PART - 1) // EDGES_PER_PART)
    parts = []
    for part in range(nparts):
        chunk = edges[part * EDGES_PER_PART : (part + 1) * EDGES_PER_PART]
        header = _ARENA_HEADER.pack(
            gen & 0xFF,
            part,
            nparts,
            int(round(math.degrees(al.theta_rad) * 100)),
            int(round(al.offset_u * 1000)) % 1000,
            int(round(al.offset_v * 1000)) % 1000,
            grid.entry_u,
            grid.entry_v,
            grid.col0,
            grid.row0,
            grid.cols,
            grid.rows,
        )
        parts.append(header + pack(chunk))
    return parts


def decode_part(payload: bytes) -> Optional[Tuple[tuple, int, int, bytes]]:
    """-> (header fields minus part/nparts, part, nparts, packed edges), or None."""
    if len(payload) < _ARENA_HEADER.size:
        return None
    f = _ARENA_HEADER.unpack_from(payload)
    if f[2] == 0 or f[1] >= f[2]:
        return None
    return (f[0],) + f[3:], f[1], f[2], payload[_ARENA_HEADER.size :]


class ArenaGridSender:
    """Holds the latest grid's payloads and hands them out one per call, round
    robin, so a GCS that starts late or misses a packet fills in. gen counts up
    each time the grid changes."""

    def __init__(self) -> None:
        self._parts: List[bytes] = []
        self._next = 0
        self._gen = 0
        self._last: Optional[Tuple[tuple, List[int]]] = None

    def update(self, grid: ArenaGrid) -> None:
        al = grid.alignment
        key = (
            (round(al.theta_rad, 4), round(al.offset_u, 3), round(al.offset_v, 3),
             grid.entry_u, grid.entry_v, grid.col0, grid.row0, grid.cols, grid.rows),
            edge_stream(grid),
        )
        if key == self._last:
            return
        self._last = key
        self._gen = (self._gen + 1) & 0xFF
        self._parts = encode_parts(grid, self._gen)
        self._next = 0

    def next_payload(self) -> Optional[bytes]:
        parts = self._parts  # one read: update() may swap it from another thread
        if not parts:
            return None
        i = self._next % len(parts)
        self._next = i + 1
        return parts[i]

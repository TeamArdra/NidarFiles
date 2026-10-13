"""SLAM map -> 1 m arena grid: alignment, edge states, cell lookup."""
import math
import random

import pytest

from nidar_autonomy.arena_grid import (
    OPEN,
    UNKNOWN,
    WALL,
    OccupancyMap,
    build_grid,
    estimate_alignment,
)

# A 6 x 5 maze in arena cells. Walls are edges: ("v", col, row) is the west
# side of (col, row); ("h", col, row) the south side. Entry (0, 0) is open to
# the south (the arena's access point).
COLS, ROWS = 6, 5


def maze_walls():
    walls = set()
    for r in range(ROWS):
        walls.add(("v", 0, r))
        walls.add(("v", COLS, r))
    for c in range(COLS):
        if c != 0:
            walls.add(("h", c, 0))
        walls.add(("h", c, ROWS))
    # inner walls: a corridor up column 0, a 2 x 2 room at cols 3-4 rows 2-3
    for r in range(0, 4):
        walls.add(("v", 1, r))
    walls.add(("h", 3, 2))
    walls.add(("h", 4, 2))
    walls.add(("h", 3, 4))
    walls.add(("h", 4, 4))
    walls.add(("v", 3, 3))  # door into the room at (3, 2)'s west side stays open
    walls.add(("v", 5, 2))
    walls.add(("v", 5, 3))
    walls.add(("h", 1, 3))
    walls.add(("h", 2, 3))
    return walls


def render(
    walls,
    theta_deg,
    start=(0.4, 0.6),
    res=0.05,
    explored=None,
    wall_half=0.03,
    noise_seed=None,
):
    """Fine OccupancyGrid of the maze as Cartographer would see it: map-frame
    origin at the drone's start (arena coords `start`, cell (0, 0)), axes
    rotated by theta_deg from the arena's. explored: set of (col, row) cells
    seen; walls next to a seen cell are seen too."""
    th = math.radians(theta_deg)
    c, s = math.cos(th), math.sin(th)
    size = 12.0
    n = int(size * 2 / res)
    ox = oy = -size
    data = [-1] * (n * n)

    def seen(col, row):
        return explored is None or (col, row) in explored

    for r in range(n):
        y = oy + (r + 0.5) * res
        for k in range(n):
            x = ox + (k + 0.5) * res
            # the arena axes are the map axes rotated by theta:
            # arena coords = R(-theta) p + start
            a = x * c + y * s + start[0]
            b = -x * s + y * c + start[1]
            col, row = math.floor(a), math.floor(b)
            if not (-0.2 < a < COLS + 0.2 and -1.0 <= b < ROWS + 0.2):
                continue
            val = -1
            ga, gb = round(a), round(b)
            if abs(a - ga) <= wall_half + res / 2 and ("v", ga, math.floor(b)) in walls:
                if seen(ga, math.floor(b)) or seen(ga - 1, math.floor(b)):
                    val = 100
            if abs(b - gb) <= wall_half + res / 2 and ("h", math.floor(a), gb) in walls:
                if seen(math.floor(a), gb) or seen(math.floor(a), gb - 1):
                    val = 100
            if val < 0 and (0 <= a < COLS and 0 <= b < ROWS) and seen(col, row):
                val = 0
            if val < 0 and 0 <= a < 1 and -1 <= b < 0:
                val = 0  # in front of the entry
            data[r * n + k] = val

    if noise_seed is not None:
        rng = random.Random(noise_seed)
        for _ in range(60):  # stray hits (dust, people at the arena edge)
            data[rng.randrange(n * n)] = 100
        for i in range(len(data)):  # patchy free space
            if data[i] == 0 and rng.random() < 0.05:
                data[i] = -1
    return OccupancyMap(data, n, n, res, ox, oy)


def check_grid(grid, walls, cells, explored=None):
    """Every edge of the given cells matches the maze. With explored given, an
    open edge into an unexplored cell is a frontier and must be UNKNOWN."""

    def want(edge, other):
        if edge in walls:
            return WALL
        if explored is not None and other not in explored and other in ALL_CELLS:
            return UNKNOWN
        return OPEN

    for col, row in cells:
        n, e, s, w = grid.cell_walls(col, row)
        assert n == want(("h", col, row + 1), (col, row + 1)), ("N", col, row)
        assert e == want(("v", col + 1, row), (col + 1, row)), ("E", col, row)
        assert s == want(("h", col, row), (col, row - 1)), ("S", col, row)
        assert w == want(("v", col, row), (col - 1, row)), ("W", col, row)


ALL_CELLS = [(c, r) for c in range(COLS) for r in range(ROWS)]


@pytest.mark.parametrize("theta_deg", [0.0, 7.0, -12.5, 30.0])
def test_alignment_recovers_rotation_and_grid_lines(theta_deg):
    m = render(maze_walls(), theta_deg)
    al = estimate_alignment(m)
    assert al is not None
    assert math.degrees(al.theta_rad) == pytest.approx(theta_deg, abs=0.5)
    # the start is at arena (0.4, 0.6) and maps to (0, 0): grid lines sit
    # 0.4 m behind and 0.6 m below it, i.e. at u = 0.6, v = 0.4 (mod 1)
    assert (al.offset_u - 0.6 + 0.5) % 1.0 - 0.5 == pytest.approx(0.0, abs=0.05)
    assert (al.offset_v - 0.4 + 0.5) % 1.0 - 0.5 == pytest.approx(0.0, abs=0.05)


@pytest.mark.parametrize("theta_deg", [0.0, 7.0, -12.5])
def test_full_maze_edges_match(theta_deg):
    walls = maze_walls()
    m = render(walls, theta_deg)
    grid = build_grid(m, estimate_alignment(m))
    check_grid(grid, walls, ALL_CELLS)


def test_entry_cell_is_origin_and_cell_of_maps_points():
    walls = maze_walls()
    m = render(walls, 7.0)
    grid = build_grid(m, estimate_alignment(m))
    assert grid.cell_of(0.0, 0.0) == (0, 0)
    # arena point (3.5, 2.5) -> the room cell (3, 2)
    th = math.radians(7.0)
    a, b = 3.5 - 0.4, 2.5 - 0.6
    x, y = a * math.cos(th) - b * math.sin(th), a * math.sin(th) + b * math.cos(th)
    assert grid.cell_of(x, y) == (3, 2)
    # the entry's south side is the open access point
    assert grid.cell_walls(0, 0)[2] == OPEN


def test_noise_does_not_change_the_maze():
    walls = maze_walls()
    m = render(walls, 4.0, noise_seed=1)
    grid = build_grid(m, estimate_alignment(m))
    check_grid(grid, walls, ALL_CELLS)


def test_unexplored_cells_stay_unknown():
    walls = maze_walls()
    explored = {(0, r) for r in range(ROWS)} | {(1, 4), (2, 4)}
    m = render(walls, 0.0, explored=explored)
    grid = build_grid(m, estimate_alignment(m))
    check_grid(grid, walls, sorted(explored), explored)
    # an unexplored cell inside the explored extent: nothing known
    assert grid.cell_walls(2, 1) == (UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN)
    # beyond the explored extent: not in the grid at all
    assert grid.cell_walls(3, 1) is None


def test_too_little_wall_gives_no_alignment():
    m = OccupancyMap([0] * 100 + [100] * 5, 105, 1, 0.05, 0.0, 0.0)
    assert estimate_alignment(m) is None


# -- over the radio -------------------------------------------------------------

from nidar_autonomy.arena_grid import (  # noqa: E402
    ARENA_GRID_TYPE,
    LOCK_MIN_POINTS,
    Alignment,
    ArenaGridSender,
    decode_part,
    edge_stream,
    encode_parts,
)
from nidar_autonomy.map_grid import TUNNEL_PAYLOAD_MAX, unpack  # noqa: E402
from nidar_autonomy.radio_telemetry import TelemetryRelay  # noqa: E402
from nidar_autonomy.telem_command_codec import MSG_ID_TUNNEL, MavlinkStreamParser  # noqa: E402


def _grid(theta_deg=7.0):
    m = render(maze_walls(), theta_deg)
    return build_grid(m, estimate_alignment(m))


def _reassemble(parts):
    """What the GCS does: header from part 0, edges from all parts in order."""
    decoded = [decode_part(p) for p in parts]
    header = decoded[0][0]
    assert all(d[0] == header for d in decoded)
    edges = []
    for fields, part, nparts, body in sorted(decoded, key=lambda d: d[1]):
        edges += unpack(body, len(body) * 4)
    gen, theta_cdeg, ou_mm, ov_mm, eu, ev, col0, row0, cols, rows = header
    count = rows * (cols + 1) + (rows + 1) * cols
    return header, edges[:count]


def test_parts_fit_a_tunnel_and_round_trip():
    grid = _grid()
    parts = encode_parts(grid, gen=5)
    assert all(len(p) <= TUNNEL_PAYLOAD_MAX for p in parts)
    header, edges = _reassemble(parts)
    gen, theta_cdeg, ou_mm, ov_mm, eu, ev, col0, row0, cols, rows = header
    assert gen == 5
    assert theta_cdeg == round(math.degrees(grid.alignment.theta_rad) * 100)
    assert (eu, ev, col0, row0, cols, rows) == (
        grid.entry_u, grid.entry_v, grid.col0, grid.row0, grid.cols, grid.rows
    )
    assert edges == edge_stream(grid)


def _full_size(edge=WALL):
    from nidar_autonomy.arena_grid import ArenaGrid

    return ArenaGrid(
        Alignment(0.0, 0.0, 0.0), 0, 0, 0, 0, 15, 15,
        [[edge] * 16 for _ in range(15)], [[OPEN] * 15 for _ in range(16)],
    )


def test_full_size_arena_is_two_parts():
    g = _full_size()
    parts = encode_parts(g, 1)
    assert len(parts) == 2
    assert _reassemble(parts)[1] == edge_stream(g)


def test_sender_round_robins_and_bumps_gen_only_on_change():
    sender = ArenaGridSender()
    assert sender.next_payload() is None
    sender.update(_full_size())
    sent = [sender.next_payload() for _ in range(4)]
    assert [decode_part(p)[1] for p in sent] == [0, 1, 0, 1]
    gen1 = decode_part(sent[0])[0][0]
    sender.update(_full_size())  # same grid: same gen
    assert decode_part(sender.next_payload())[0][0] == gen1
    sender.update(_full_size(edge=UNKNOWN))  # changed: new gen, from part 0
    p = decode_part(sender.next_payload())
    assert (p[0][0], p[1]) == ((gen1 + 1) & 0xFF, 0)


def test_near_search_keeps_a_well_supported_alignment():
    m = render(maze_walls(), 7.0)
    first = estimate_alignment(m)
    assert first.support >= LOCK_MIN_POINTS
    again = estimate_alignment(m, near=first)
    assert math.degrees(again.theta_rad) == pytest.approx(7.0, abs=0.5)
    # a weak previous estimate is ignored: full search again
    weak = Alignment(math.radians(-30.0), 0.0, 0.0, support=10)
    assert math.degrees(estimate_alignment(m, near=weak).theta_rad) == pytest.approx(7.0, abs=0.5)


class _Seq:
    def __init__(self):
        self.n = 0

    def __call__(self):
        self.n = (self.n + 1) & 0xFF
        return self.n


def _arena_tunnels(raw):
    out = []
    for f in MavlinkStreamParser().feed(b"".join(raw)):
        if f.msgid == MSG_ID_TUNNEL and int.from_bytes(f.payload[:2], "little") == ARENA_GRID_TYPE:
            out.append(f)
    return out


def test_relay_sends_arena_parts_at_its_rate_and_stops_when_stale():
    relay = TelemetryRelay(_Seq(), 0.0, lidar_rate_hz=0, map_rate_hz=0, arena_rate_hz=1.0)
    relay.update_arena(0.0, _grid())
    sent = 0
    for k in range(20):  # 10 s at 2 Hz ticks
        sent += len(_arena_tunnels(relay.tick(k * 0.5)))
    assert 9 <= sent <= 11
    assert _arena_tunnels(relay.tick(60.0)) == []  # no fresh grid for 60 s


def test_relay_sends_nothing_with_arena_off():
    relay = TelemetryRelay(_Seq(), 0.0, lidar_rate_hz=0, map_rate_hz=0, arena_rate_hz=0)
    relay.update_arena(0.0, _grid())
    assert _arena_tunnels(relay.tick(1.0)) == []

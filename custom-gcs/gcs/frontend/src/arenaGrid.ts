// Geometry of the 1 m arena grid (types.ts ArenaGrid): arena <-> map frame,
// which cell a point is in, and the edges as map-frame line segments for the
// renderer. Pure functions, no canvas.
import type { ArenaEdge, ArenaGrid } from "./types";

export const EDGE_UNKNOWN = 0;
export const EDGE_OPEN = 1;
export const EDGE_WALL = 2;

/** Arena-frame (u, v) metres -> map-frame (x, y) metres. */
export function arenaToMap(arena: ArenaGrid, u: number, v: number): [number, number] {
  const t = (arena.theta_deg * Math.PI) / 180;
  const c = Math.cos(t);
  const s = Math.sin(t);
  return [u * c - v * s, u * s + v * c];
}

/** Map-frame corner of grid line (col, row): the south-west corner of that cell. */
export function cornerToMap(arena: ArenaGrid, col: number, row: number): [number, number] {
  return arenaToMap(arena, arena.offset_u + arena.entry_u + col, arena.offset_v + arena.entry_v + row);
}

/** The cell (relative to the take-off cell) a map-frame point is in. */
export function cellOf(arena: ArenaGrid, x: number, y: number): [number, number] {
  const t = (arena.theta_deg * Math.PI) / 180;
  const c = Math.cos(t);
  const s = Math.sin(t);
  const u = x * c + y * s;
  const v = -x * s + y * c;
  return [Math.floor(u - arena.offset_u) - arena.entry_u, Math.floor(v - arena.offset_v) - arena.entry_v];
}

export interface ArenaSegment {
  from: [number, number];
  to: [number, number];
  state: ArenaEdge;
}

/** Every edge of the grid as a map-frame segment. */
export function arenaSegments(arena: ArenaGrid): ArenaSegment[] {
  const out: ArenaSegment[] = [];
  for (let r = 0; r < arena.rows; r++) {
    for (let k = 0; k <= arena.cols; k++) {
      const col = arena.col0 + k;
      const row = arena.row0 + r;
      out.push({ from: cornerToMap(arena, col, row), to: cornerToMap(arena, col, row + 1), state: arena.v_edges[r][k] });
    }
  }
  for (let k = 0; k <= arena.rows; k++) {
    for (let c = 0; c < arena.cols; c++) {
      const col = arena.col0 + c;
      const row = arena.row0 + k;
      out.push({ from: cornerToMap(arena, col, row), to: cornerToMap(arena, col + 1, row), state: arena.h_edges[k][c] });
    }
  }
  return out;
}

/** Edge counts by state, for the panel's summary line. */
export function arenaCounts(arena: ArenaGrid): { walls: number; open: number; unknown: number } {
  let walls = 0;
  let open = 0;
  let unknown = 0;
  for (const row of [...arena.v_edges, ...arena.h_edges]) {
    for (const e of row) {
      if (e === EDGE_WALL) walls++;
      else if (e === EDGE_OPEN) open++;
      else unknown++;
    }
  }
  return { walls, open, unknown };
}

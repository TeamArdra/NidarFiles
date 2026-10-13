import { describe, expect, it } from "vitest";
import { arenaCounts, arenaSegments, arenaToMap, cellOf, cornerToMap } from "./arenaGrid";
import type { ArenaGrid } from "./types";

// 2 cols x 1 row, take-off cell (0, 0) at grid line (-1, 0), rotated 90°.
const ARENA: ArenaGrid = {
  theta_deg: 90,
  offset_u: 0.5,
  offset_v: 0.25,
  entry_u: -1,
  entry_v: 0,
  col0: 0,
  row0: 0,
  cols: 2,
  rows: 1,
  v_edges: [[2, 1, 2]],
  h_edges: [
    [2, 0],
    [2, 2],
  ],
};

function close(a: [number, number], b: [number, number]) {
  expect(a[0]).toBeCloseTo(b[0], 6);
  expect(a[1]).toBeCloseTo(b[1], 6);
}

describe("arenaGrid", () => {
  it("rotates arena coordinates into the map frame", () => {
    close(arenaToMap(ARENA, 1, 0), [0, 1]);
    close(arenaToMap(ARENA, 0, 1), [-1, 0]);
  });

  it("puts cell (0, 0)'s corner at the offsets plus the entry grid line", () => {
    // u = 0.5 - 1 = -0.5, v = 0.25 -> map (-0.25, -0.5)
    close(cornerToMap(ARENA, 0, 0), [-0.25, -0.5]);
  });

  it("finds the cell of a map point, inverse of cornerToMap", () => {
    const [x, y] = arenaToMap(ARENA, 0.5 - 1 + 1.5, 0.25 + 0.5); // middle of cell (1, 0)
    expect(cellOf(ARENA, x, y)).toEqual([1, 0]);
    const [x0, y0] = arenaToMap(ARENA, 0.5 - 1 + 0.5, 0.25 + 0.5); // middle of cell (0, 0)
    expect(cellOf(ARENA, x0, y0)).toEqual([0, 0]);
  });

  it("lists every edge once with its state", () => {
    const segs = arenaSegments(ARENA);
    expect(segs).toHaveLength(3 + 4);
    expect(segs.map((s) => s.state)).toEqual([2, 1, 2, 2, 0, 2, 2]);
    close(segs[0].from, cornerToMap(ARENA, 0, 0));
    close(segs[0].to, cornerToMap(ARENA, 0, 1));
  });

  it("counts edges by state", () => {
    expect(arenaCounts(ARENA)).toEqual({ walls: 5, open: 1, unknown: 1 });
  });
});

// Shared occupancy-grid canvas renderer -- extracted from MapPanel.tsx so
// SimulationPanel.tsx can render the SAME kind of map (occupancy +
// coverage overlay + planned path + drone position + target) without
// duplicating the drawing logic. Two callers, one renderer -- the
// simulation is a consumer of the same visualization, not a second
// implementation of it.
import { EDGE_OPEN, EDGE_WALL, arenaSegments, cellOf, cornerToMap } from "./arenaGrid";
import type { ArenaGrid, CoverageResponse, FrontierPointResponse, MapResponse, PathResponse } from "./types";

// Palette matches tailwind.config.js's dark operator-panel colors --
// canvas fills can't reference Tailwind classes, so the same hex values
// are duplicated here deliberately (see tailwind.config.js comment: the
// visual language should stay one system).
export const COLOR_UNKNOWN = "#26313d"; // border
export const COLOR_FREE = "#0b0f14"; // bg
export const COLOR_OCCUPIED = "#e6edf3"; // text
export const COLOR_SEARCHED = "rgba(46, 160, 67, 0.35)"; // ok, translucent
export const COLOR_PATH = "#388bfd"; // accent
export const COLOR_DRONE = "#388bfd"; // accent
export const COLOR_TARGET = "#d29922"; // warn
export const COLOR_FRONTIER = "#a371f7"; // purple, distinct from path/target/drone
// 1 m arena grid: walls must stand out from the raw SLAM cells (white)
export const COLOR_GRID_WALL = "#f0883e"; // orange
export const COLOR_GRID_OPEN = "#3fb950"; // ok green
export const COLOR_GRID_UNKNOWN = "rgba(139, 148, 158, 0.3)"; // dim, faint
export const COLOR_GRID_ENTRY = "rgba(56, 139, 253, 0.22)"; // accent, translucent

// nav_msgs/OccupancyGrid cell semantics (standard for /map and
// /simulation/map; /coverage_grid and /simulation/coverage_grid reuse the
// same encoding with different meaning -- see
// CHECKPOINT/docs/gcs_telemetry_contract.md).
const UNKNOWN = -1;
const OCCUPIED_THRESHOLD = 65;

function worldToCanvas(
  x: number,
  y: number,
  originX: number,
  originY: number,
  resolution: number,
  cellPx: number,
): [number, number] {
  return [((x - originX) / resolution) * cellPx, ((y - originY) / resolution) * cellPx];
}

export interface RenderMapOptions {
  map: MapResponse;
  coverage?: CoverageResponse | null;
  path?: PathResponse | null;
  dronePosition?: { x: number; y: number } | null;
  /** Heading in degrees, counter-clockwise from +x (map frame); draws an arrow. */
  droneYawDeg?: number | null;
  target?: [number, number] | null;
  frontiers?: FrontierPointResponse[] | null;
  originX?: number;
  originY?: number;
  cellPx?: number;
  /** The 1 m arena grid (radio ARENA_GRID); drawn over the SLAM cells. */
  arena?: ArenaGrid | null;
}

/** Draws one frame of the occupancy map (+ overlays) onto `canvas`. Sizes
 * the canvas to fit the grid at `cellPx` pixels/cell, DPR-aware. Callers
 * are responsible for polling fresh data and calling this again -- this
 * function has no state or timers of its own. */
export function renderOccupancyMapCanvas(canvas: HTMLCanvasElement, options: RenderMapOptions): void {
  const {
    map,
    coverage,
    path,
    dronePosition,
    droneYawDeg,
    target,
    frontiers,
    originX = 0,
    originY = 0,
    cellPx = 8,
    arena,
  } = options;
  if (map.data == null || map.width == null || map.height == null || map.resolution == null) {
    return;
  }

  const width = map.width * cellPx;
  const height = map.height * cellPx;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = width * dpr;
  canvas.height = height * dpr;
  canvas.style.width = `${width}px`;
  canvas.style.height = `${height}px`;

  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  ctx.scale(dpr, dpr);

  // -- occupancy grid --
  for (let row = 0; row < map.height; row++) {
    for (let col = 0; col < map.width; col++) {
      const v = map.data[row * map.width + col];
      ctx.fillStyle = v === UNKNOWN ? COLOR_UNKNOWN : v >= OCCUPIED_THRESHOLD ? COLOR_OCCUPIED : COLOR_FREE;
      // Canvas Y grows downward; occupancy grid Y grows "up" (world
      // convention) -- flip the row so north is up on screen.
      ctx.fillRect(col * cellPx, (map.height - 1 - row) * cellPx, cellPx, cellPx);
    }
  }

  // -- coverage overlay (searched cells only; leaves unsearched/unknown transparent) --
  if (coverage?.data != null && coverage.width != null && coverage.height != null) {
    const covCellPx = (map.width * cellPx) / coverage.width;
    const covCellPy = (map.height * cellPx) / coverage.height;
    for (let row = 0; row < coverage.height; row++) {
      for (let col = 0; col < coverage.width; col++) {
        const v = coverage.data[row * coverage.width + col];
        if (v >= OCCUPIED_THRESHOLD) {
          ctx.fillStyle = COLOR_SEARCHED;
          ctx.fillRect(col * covCellPx, (coverage.height - 1 - row) * covCellPy, covCellPx, covCellPy);
        }
      }
    }
  }

  // -- 1 m arena grid: take-off cell, drone's cell, then every edge --
  if (arena) {
    const res = map.resolution;
    const toCanvas = ([x, y]: [number, number]): [number, number] => {
      const [px, py] = worldToCanvas(x, y, originX, originY, res, cellPx);
      return [px, height - py];
    };
    const cellPath = (col: number, row: number) => {
      const corners = [
        cornerToMap(arena, col, row),
        cornerToMap(arena, col + 1, row),
        cornerToMap(arena, col + 1, row + 1),
        cornerToMap(arena, col, row + 1),
      ].map(toCanvas);
      ctx.beginPath();
      corners.forEach(([px, py], i) => (i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py)));
      ctx.closePath();
    };
    cellPath(0, 0);
    ctx.fillStyle = COLOR_GRID_ENTRY;
    ctx.fill();
    if (dronePosition) {
      const [col, row] = cellOf(arena, dronePosition.x, dronePosition.y);
      cellPath(col, row);
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = COLOR_DRONE;
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.setLineDash([]);
    }
    // unknown first, walls last, so walls are never painted over
    const order = [0, EDGE_OPEN, EDGE_WALL];
    const segments = arenaSegments(arena).sort((a, b) => order.indexOf(a.state) - order.indexOf(b.state));
    for (const seg of segments) {
      const [x0, y0] = toCanvas(seg.from);
      const [x1, y1] = toCanvas(seg.to);
      ctx.strokeStyle =
        seg.state === EDGE_WALL ? COLOR_GRID_WALL : seg.state === EDGE_OPEN ? COLOR_GRID_OPEN : COLOR_GRID_UNKNOWN;
      ctx.lineWidth = seg.state === EDGE_WALL ? 3 : 1;
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
    }
  }

  // -- planned path --
  if (path?.points && path.points.length > 1) {
    ctx.strokeStyle = COLOR_PATH;
    ctx.lineWidth = 2;
    ctx.beginPath();
    path.points.forEach((pt, i) => {
      const [px, py] = worldToCanvas(pt.x, pt.y, originX, originY, map.resolution!, cellPx);
      const canvasY = height - py;
      if (i === 0) ctx.moveTo(px, canvasY);
      else ctx.lineTo(px, canvasY);
    });
    ctx.stroke();
  }

  // -- drone position --
  if (dronePosition) {
    const [dx, dy] = worldToCanvas(dronePosition.x, dronePosition.y, originX, originY, map.resolution, cellPx);
    ctx.fillStyle = COLOR_DRONE;
    ctx.beginPath();
    ctx.arc(dx, height - dy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1;
    ctx.stroke();
    if (droneYawDeg != null) {
      const a = (droneYawDeg * Math.PI) / 180;
      const len = 14;
      ctx.strokeStyle = COLOR_DRONE;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(dx, height - dy);
      // canvas y grows downward: a counter-clockwise world angle is -sin on screen
      ctx.lineTo(dx + len * Math.cos(a), height - dy - len * Math.sin(a));
      ctx.stroke();
    }
  }

  // -- current frontier/target --
  if (target) {
    const [tx, ty] = worldToCanvas(target[0], target[1], originX, originY, map.resolution, cellPx);
    ctx.strokeStyle = COLOR_TARGET;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(tx, height - ty, 6, 0, Math.PI * 2);
    ctx.stroke();
  }

  // -- frontier candidates (exploration boundary points, /frontiers) --
  if (frontiers && frontiers.length > 0) {
    ctx.fillStyle = COLOR_FRONTIER;
    for (const f of frontiers) {
      const [fx, fy] = worldToCanvas(f.x, f.y, originX, originY, map.resolution, cellPx);
      ctx.beginPath();
      ctx.arc(fx, height - fy, 3, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}

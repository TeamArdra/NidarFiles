import { useEffect, useRef, useState } from "react";
import Panel from "./Panel";
import { getCoverage, getFrontiers, getMap, getPath } from "../api";
import { arenaCounts, cellOf } from "../arenaGrid";
import { renderOccupancyMapCanvas } from "../mapRender";
import type {
  CoverageResponse,
  FrontiersResponse,
  MapResponse,
  PathResponse,
  TelemetryResponse,
} from "../types";

const POLL_INTERVAL_MS = 1000;

// Extension point: /api/map has no real data until onboard-autonomy's
// SLAM/mapping pipeline is running (AUTONOMY_ROADMAP.md Phase 4/5 -- see
// the NIDAR Autonomy Migration report, CHECKPOINT/CURRENT_STATE.md, for
// current status) -- renders an explicit "no map yet" placeholder rather
// than a fake/demo grid, same posture as before this panel had real
// rendering logic. Once /api/map has real data (or the sim/rosbridge_sim
// synthetic feed is used for testing), this renders it for real: occupied/
// free/unknown cells, a coverage overlay, the planned path, the drone's
// current position, and the current frontier/target -- see
// custom-gcs/CLAUDE.md "never invent" for why nothing here is fabricated
// beyond what the backend actually reports.
//
// This is the REAL-mission map, sourced from /api/map (the real /map
// topic) -- for the separate simulation map, see SimulationPanel.tsx,
// which renders via the same renderOccupancyMapCanvas() but from
// /api/simulation/map. The two never share data.
export default function MapPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const [map, setMap] = useState<MapResponse | null>(null);
  const [coverage, setCoverage] = useState<CoverageResponse | null>(null);
  const [path, setPath] = useState<PathResponse | null>(null);
  const [frontiers, setFrontiers] = useState<FrontiersResponse | null>(null);
  const [error, setError] = useState<string | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    let mounted = true;

    async function poll() {
      try {
        const [m, c, p, f] = await Promise.all([getMap(), getCoverage(), getPath(), getFrontiers()]);
        if (mounted) {
          setMap(m);
          setCoverage(c);
          setPath(p);
          setFrontiers(f);
          setError(null);
        }
      } catch (e) {
        if (mounted) setError(e instanceof Error ? e.message : String(e));
      }
    }

    poll();
    const id = setInterval(poll, POLL_INTERVAL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  const hasMap = map?.data != null && map.data.length > 0 && map.width != null && map.height != null;
  const arena = map?.arena ?? null;
  const counts = arena ? arenaCounts(arena) : null;
  const robotCell = arena && map?.robot ? cellOf(arena, map.robot.x, map.robot.y) : null;

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !hasMap || !map) return;

    renderOccupancyMapCanvas(canvas, {
      map,
      coverage,
      path,
      // Map origin is always (0,0) here: /api/map doesn't carry the
      // origin separately (see MapResponse) -- it's derived from the
      // telemetry mapping block when available, defaulting to (0,0).
      // Radio SLAM map: its own origin and the SLAM pose; otherwise the
      // telemetry mapping block / FCU pose as before.
      originX: map.origin_x ?? telemetry?.mapping?.origin_x ?? 0,
      originY: map.origin_y ?? telemetry?.mapping?.origin_y ?? 0,
      dronePosition: map.robot ?? telemetry?.pose?.position ?? null,
      droneYawDeg: map.robot?.yaw_deg ?? null,
      cellPx: Math.max(3, Math.min(8, Math.floor(720 / Math.max(map.width ?? 1, map.height ?? 1)))),
      target: telemetry?.autonomy?.target ?? telemetry?.navigation?.target ?? null,
      frontiers: frontiers?.points ?? null,
      arena: map.arena ?? null,
    });
  }, [map, coverage, path, frontiers, telemetry, hasMap]);

  return (
    <Panel title="Map" className="col-span-full">
      {error && <div className="text-bad text-xs mb-2">map fetch failed: {error}</div>}
      {hasMap ? (
        <div>
          <div className="text-xs text-dim mb-2">
            {map!.width} × {map!.height} cells, resolution {map!.resolution} m/cell
            {map!.age_s != null && ` — updated ${map!.age_s} s ago`}
            {map!.robot && ` — position ${map!.robot.x.toFixed(2)}, ${map!.robot.y.toFixed(2)} m, heading ${Math.round(map!.robot.yaw_deg)}°`}
            {telemetry?.mapping?.explored_pct != null && ` — ${telemetry.mapping.explored_pct}% searched`}
          </div>
          <div className="text-xs text-dim mb-2" data-testid="arena-grid-summary">
            {arena && counts ? (
              <>
                1 m grid: {arena.cols} × {arena.rows} cells — {counts.walls} walls, {counts.open} open,{" "}
                {counts.unknown} unknown
                {robotCell && ` — drone in cell (${robotCell[0]}, ${robotCell[1]})`}
                {` — cell (0, 0) = take-off, rotated ${arena.theta_deg.toFixed(1)}°`}
                {arena.age_s != null && arena.age_s > 10 && ` — grid ${arena.age_s} s old`}
              </>
            ) : (
              "1 m grid: not aligned yet — the Jetson needs a few walls in the map first"
            )}
          </div>
          <div className="overflow-auto border border-border rounded">
            <canvas ref={canvasRef} data-testid="map-canvas" />
          </div>
        </div>
      ) : (
        <div className="text-dim text-xs">
          No map data yet — start_jetson.sh runs the map by default (LiDAR + Cartographer SLAM;
          not with --no-map / --no-lidar). It arrives over the radio and grows as the LiDAR moves.
          See the Components panel for what is connected.
        </div>
      )}
    </Panel>
  );
}

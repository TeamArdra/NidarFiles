import { useEffect, useState } from "react";
import Panel from "./Panel";
import { getLidar } from "../api";
import type { LidarResponse } from "../types";

// Live LiDAR (RPLIDAR A2 on the Jetson) as received over the radio:
// 72 sectors x 5 deg, nearest return per sector. Top of the view = the
// drone's nose; angles run clockwise. Read-only display.

const POLL_MS = 1000;
const SIZE = 260;
const C = SIZE / 2;
const R = SIZE / 2 - 14;
const SCALES_M = [1, 2, 3, 4, 6, 8, 12, 16];

function pick_scale(maxSeenM: number): number {
  return SCALES_M.find((s) => s >= maxSeenM) ?? SCALES_M[SCALES_M.length - 1];
}

export interface Ray {
  angleDeg: number;
  distanceM: number;
}

export function rays(scan: LidarResponse): Ray[] {
  return scan.distances_cm.flatMap((d, i) =>
    d == null ? [] : [{ angleDeg: scan.angle_offset_deg + i * scan.increment_deg, distanceM: d / 100 }],
  );
}

function xy(angleDeg: number, r: number): [number, number] {
  const a = (angleDeg * Math.PI) / 180;
  return [C + r * Math.sin(a), C - r * Math.cos(a)];
}

export default function LidarPanel() {
  const [scan, setScan] = useState<LidarResponse | null>(null);

  useEffect(() => {
    let mounted = true;
    async function poll() {
      try {
        const s = await getLidar();
        if (mounted) setScan(s);
      } catch {
        if (mounted) setScan(null);
      }
    }
    poll();
    const id = setInterval(poll, POLL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  const pts = scan?.available ? rays(scan) : [];
  const scale = pick_scale(Math.max(0, ...pts.map((p) => p.distanceM)));
  const nearest = pts.reduce<Ray | null>((best, p) => (best == null || p.distanceM < best.distanceM ? p : best), null);
  const toR = (m: number) => (Math.min(m, scale) / scale) * R;

  // Outline between neighbouring sectors that both have a return.
  const segments: string[] = [];
  if (scan?.available) {
    const n = scan.distances_cm.length;
    for (let i = 0; i < n; i++) {
      const a = scan.distances_cm[i];
      const b = scan.distances_cm[(i + 1) % n];
      if (a == null || b == null) continue;
      const [x1, y1] = xy(scan.angle_offset_deg + i * scan.increment_deg, toR(a / 100));
      const [x2, y2] = xy(scan.angle_offset_deg + (i + 1) * scan.increment_deg, toR(b / 100));
      segments.push(`M${x1.toFixed(1)},${y1.toFixed(1)}L${x2.toFixed(1)},${y2.toFixed(1)}`);
    }
  }

  return (
    <Panel title="LiDAR (live, over the radio)">
      {!scan?.available ? (
        <div className="text-dim text-sm">No LiDAR scan — check the LiDAR is plugged into the Jetson (start_jetson.sh runs it unless --no-lidar).</div>
      ) : (
        <>
          <svg viewBox={`0 0 ${SIZE} ${SIZE}`} className="w-full max-w-[320px] mx-auto block" role="img" aria-label="LiDAR scan">
            {[0.25, 0.5, 0.75, 1].map((f) => (
              <g key={f}>
                <circle cx={C} cy={C} r={R * f} fill="none" stroke="#26313d" strokeWidth={1} />
                <text x={C + 3} y={C - R * f + 10} fontSize={9} fill="#8b98a5">
                  {(scale * f).toFixed(scale * f < 1 ? 2 : 1)} m
                </text>
              </g>
            ))}
            <line x1={C} y1={C - R} x2={C} y2={C + R} stroke="#1d252d" />
            <line x1={C - R} y1={C} x2={C + R} y2={C} stroke="#1d252d" />
            <text x={C} y={10} fontSize={9} fill="#8b98a5" textAnchor="middle">FRONT</text>
            <path d={segments.join("")} stroke="#2ea043" strokeOpacity={0.5} strokeWidth={1.5} fill="none" />
            {pts.map((p, i) => {
              const [x, y] = xy(p.angleDeg, toR(p.distanceM));
              const near = nearest != null && p === nearest;
              return <circle key={i} cx={x} cy={y} r={near ? 4 : 2.5} fill={near ? "#d29922" : "#2ea043"} />;
            })}
            <polygon points={`${C},${C - 9} ${C - 6},${C + 6} ${C + 6},${C + 6}`} fill="#388bfd" />
          </svg>
          <div className="mt-2 text-xs text-dim flex flex-wrap gap-x-3">
            <span>updated {scan.age_s ?? "?"} s ago</span>
            <span>
              {pts.length}/{scan.distances_cm.length} sectors × {scan.increment_deg}°
            </span>
            {nearest && (
              <span className="text-warn">
                nearest {nearest.distanceM.toFixed(2)} m at {Math.round(((nearest.angleDeg % 360) + 360) % 360)}°
              </span>
            )}
          </div>
        </>
      )}
    </Panel>
  );
}

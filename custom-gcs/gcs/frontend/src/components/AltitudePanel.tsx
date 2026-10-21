import { useEffect, useRef, useState } from "react";
import Panel, { Row } from "./Panel";
import { getAltitude } from "../api";

// Live height above the take-off point, from every LOCAL_POSITION_NED the
// FCU sends over the radio (5 Hz, start_jetson.sh), and the downward
// rangefinder's distance (blue). The backend keeps the samples, so polling
// 4x a second never drops one; the graph shows the last WINDOW_S seconds.
// Height is relative to where the FCU last armed (the baro's absolute value
// drifts by metres indoors); the rangefinder is the distance to the floor
// as measured -- the two should agree once the EKF uses it for height.
// Read-only.

const POLL_MS = 250;
const WINDOW_S = 60;
const W = 300;
const H = 120;

export interface Sample {
  t: number; // backend clock, s
  h: number; // height above take-off (or raw z before the first arm), m
  vz: number | null; // climb rate, m/s (up = +)
}

export function toSamples(raw: Array<[number, number, number | null]>, groundZ: number | null): Sample[] {
  return raw.map(([t, z, vz]) => ({ t, h: groundZ == null ? z : z - groundZ, vz }));
}

/** SVG polyline points for the samples in the last WINDOW_S before `now`. */
export function polyline(samples: Sample[], now: number, top: number): string {
  const bottom = Math.min(0, ...samples.map((s) => s.h));
  const span = Math.max(top - bottom, 0.1);
  return samples
    .filter((s) => s.t >= now - WINDOW_S)
    .map((s) => {
      const x = W - ((now - s.t) / WINDOW_S) * W;
      const y = H - ((s.h - bottom) / span) * H;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(" ");
}

export default function AltitudePanel() {
  const [samples, setSamples] = useState<Sample[]>([]);
  const [range, setRange] = useState<Sample[]>([]);
  const [now, setNow] = useState<number | null>(null);
  const [relative, setRelative] = useState(false);
  const since = useRef<number | null>(null);
  const groundZ = useRef<number | null>(null);

  useEffect(() => {
    let mounted = true;
    const poll = () =>
      getAltitude(since.current)
        .then((a) => {
          if (!mounted || !a.available || a.now == null) return;
          setNow(a.now);
          setRelative(a.ground_z != null);
          if (a.ground_z !== groundZ.current) {
            // a new arm moved the reference: fetch the whole window again on
            // the next poll and draw it relative to the new take-off point
            groundZ.current = a.ground_z;
            since.current = null;
            setSamples([]);
            setRange([]);
            return;
          }
          const lastT = Math.max(
            a.samples.length ? a.samples[a.samples.length - 1][0] : -Infinity,
            a.range?.length ? a.range[a.range.length - 1][0] : -Infinity,
          );
          if (Number.isFinite(lastT)) since.current = lastT;
          const fresh = toSamples(a.samples, a.ground_z);
          const freshRange = (a.range ?? []).map(([t, m]) => ({ t, h: m, vz: null }));
          const keep = (s: Sample) => s.t >= (a.now ?? 0) - WINDOW_S;
          setSamples((prev) => [...prev, ...fresh].filter(keep));
          setRange((prev) => [...prev, ...freshRange].filter(keep));
        })
        .catch(() => undefined);
    poll();
    const id = setInterval(poll, POLL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  const last = samples[samples.length - 1];
  const fresh = last != null && now != null && now - last.t < 2;
  const lastRange = range[range.length - 1];
  const rangeFresh = lastRange != null && now != null && now - lastRange.t < 2;
  const top = Math.max(1, ...samples.map((s) => s.h + 0.2), ...range.map((s) => s.h + 0.2));
  return (
    <Panel title="Altitude (live)">
      <div className="flex items-baseline gap-4 mb-1">
        <span className="text-2xl tabular-nums font-semibold" data-testid="alt-now">
          {fresh ? `${last.h.toFixed(2)} m` : "—"}
        </span>
        <span className="text-xs text-dim tabular-nums" data-testid="alt-vz">
          {fresh && last.vz != null ? `${last.vz >= 0 ? "↑" : "↓"} ${Math.abs(last.vz).toFixed(2)} m/s` : ""}
        </span>
        <span className="ml-auto text-sm tabular-nums text-[#58a6ff]" data-testid="alt-range">
          rangefinder {rangeFresh ? `${lastRange.h.toFixed(2)} m` : "—"}
        </span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`} className="w-full h-[120px] bg-[#0d1217] rounded" role="img" aria-label="altitude graph">
        {[0.25, 0.5, 0.75].map((f) => (
          <line key={f} x1={0} x2={W} y1={H * f} y2={H * f} stroke="#1d252d" strokeWidth={1} />
        ))}
        {now != null && range.length > 1 && (
          <polyline points={polyline(range, now, top)} fill="none" stroke="#58a6ff" strokeWidth={1.5} />
        )}
        {now != null && samples.length > 1 && (
          <polyline points={polyline(samples, now, top)} fill="none" stroke="#3fb950" strokeWidth={2} />
        )}
      </svg>
      <Row label="Scale">0 – {top.toFixed(1)} m, last {WINDOW_S} s</Row>
      <Row label="Reference">{relative ? "height above take-off" : "raw FCU height (not armed yet)"}</Row>
      <Row label="Lines">green = EKF height, blue = rangefinder</Row>
    </Panel>
  );
}

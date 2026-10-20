import { useEffect, useState } from "react";
import Panel, { Row } from "./Panel";
import Pill, { ConnPill } from "./Pill";
import { getComponents } from "../api";
import { fmtNum } from "../format";
import type { ComponentState, ComponentStatus, FcuStateResponse, TelemetryResponse } from "../types";

// Three missed 1 Hz heartbeats (matching the 1000ms poll cadence in
// useTelemetry) is treated as stale, same threshold whether the staleness
// comes from a null heartbeat_age_s (never received) or an old one.
const STALE_HEARTBEAT_S = 3;
const POLL_MS = 1000;

// Drone parts below the two link rows (the links themselves come from
// /api/telemetry), with the short names the compact grid uses.
const PART_NAMES: Record<string, string> = {
  battery: "Battery",
  position: "Position (flow)",
  orientation: "IMU",
  lidar: "LiDAR",
  map: "SLAM map",
};

const DOT: Record<ComponentState, string> = { ok: "bg-ok", lost: "bg-bad", missing: "bg-dim" };
const STATE_TEXT: Record<ComponentState, string> = { ok: "connected", lost: "lost", missing: "not found" };

// Modes the Pixhawk flies by itself; every other armed mode is the RC pilot.
const SELF_FLOWN = new Set(["LAND", "RTL", "SMART_RTL", "AUTO_RTL", "BRAKE"]);

export interface Control {
  mode: string;
  who: string;
  kind: "ok" | "warn" | "unknown";
}

// Who is flying the drone right now, from the FCU's mode and armed state.
export function control(fcu: FcuStateResponse | undefined): Control | null {
  if (!fcu?.connected || !fcu.mode) return null;
  if (!fcu.armed) return { mode: fcu.mode, who: "disarmed", kind: "unknown" };
  if (fcu.mode === "GUIDED") return { mode: fcu.mode, who: "Jetson (autonomous)", kind: "ok" };
  if (SELF_FLOWN.has(fcu.mode)) return { mode: fcu.mode, who: "Pixhawk (failsafe/landing)", kind: "warn" };
  return { mode: fcu.mode, who: "RC pilot", kind: "warn" };
}

function useParts(): ComponentStatus[] | null {
  const [parts, setParts] = useState<ComponentStatus[] | null>(null);
  useEffect(() => {
    let mounted = true;
    async function poll() {
      try {
        const d = await getComponents();
        if (mounted) setParts(d.available ? d.components.filter((c) => c.key in PART_NAMES) : null);
      } catch {
        if (mounted) setParts(null);
      }
    }
    poll();
    const id = setInterval(poll, POLL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);
  return parts;
}

export default function ConnectionPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const heartbeatAge = telemetry?.heartbeat_age_s ?? null;
  const isStale = heartbeatAge === null || heartbeatAge > STALE_HEARTBEAT_S;
  const ctrl = control(telemetry?.fcu);
  const parts = useParts();

  return (
    <Panel title="Connection">
      {/* Not a Row: the heartbeat text must wrap under the pill, never squeeze the label. */}
      <div className="flex justify-between gap-2 py-1 border-b border-b-[#1d252d] text-[13px]">
        <span className="text-dim whitespace-nowrap">GCS ↔ Jetson</span>
        <span className="flex flex-wrap justify-end items-center gap-x-2 tabular-nums">
          <span className={`text-xs ${isStale ? "text-warn" : "text-dim"}`}>
            {heartbeatAge === null ? "no heartbeat received" : `${fmtNum(heartbeatAge, 1)} s`}
          </span>
          <ConnPill value={telemetry?.connected ?? null} />
        </span>
      </div>
      <Row label="Jetson ↔ Pixhawk">
        <ConnPill value={telemetry?.fcu.connected ?? null} />
      </Row>
      <Row label="Control">
        {ctrl ? (
          <>
            <span className="text-xs text-dim mr-2">{ctrl.who}</span>
            <Pill kind={ctrl.kind}>{ctrl.mode}</Pill>
          </>
        ) : (
          <span className="text-dim">—</span>
        )}
      </Row>
      {parts && telemetry?.connected ? (
        <div className="grid grid-cols-2 gap-x-3 gap-y-1 pt-2 text-[13px]">
          {parts.map((p) => (
            <div
              key={p.key}
              className="flex items-center gap-1.5 min-w-0"
              aria-label={`${PART_NAMES[p.key]}: ${STATE_TEXT[p.status]}`}
              title={`${p.name}: ${STATE_TEXT[p.status]}${p.status === "lost" ? ` (${fmtNum(p.age_s, 1)} s ago)` : ""}`}
            >
              <span className={`inline-block w-2 h-2 rounded-full shrink-0 ${DOT[p.status]}`} />
              <span className={p.status === "ok" ? "" : "text-dim"}>{PART_NAMES[p.key]}</span>
              {p.status === "ok" && p.key === "battery" && p.detail && (
                <span className="text-dim text-xs truncate">{p.detail}</span>
              )}
            </div>
          ))}
        </div>
      ) : (
        <div className="pt-2 text-xs text-dim">No data from the drone's parts yet.</div>
      )}
    </Panel>
  );
}

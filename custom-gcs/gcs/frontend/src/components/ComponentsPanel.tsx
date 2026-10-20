import { useEffect, useState } from "react";
import Panel from "./Panel";
import Pill from "./Pill";
import { getComponents } from "../api";
import { fmtNum } from "../format";
import type { ComponentState, ComponentStatus, ComponentsResponse } from "../types";

// What on the drone the GCS is hearing from right now, one row per part
// (radio link, Pixhawk, battery monitor, position estimate, IMU, LiDAR,
// SLAM map). Built by the backend from what keeps arriving over the radio
// -- read-only, nothing here commands the drone.

const POLL_MS = 1000;

const PILLS: Record<ComponentState, { kind: "ok" | "bad" | "unknown"; label: string }> = {
  ok: { kind: "ok", label: "CONNECTED" },
  lost: { kind: "bad", label: "LOST" },
  missing: { kind: "unknown", label: "NOT FOUND" },
};

function note(c: ComponentStatus): string {
  if (c.status === "ok") return c.detail ?? "";
  if (c.status === "lost") return `nothing for ${fmtNum(c.age_s, 1)} s`;
  return "never heard";
}

export default function ComponentsPanel() {
  const [data, setData] = useState<ComponentsResponse | null>(null);

  useEffect(() => {
    let mounted = true;
    async function poll() {
      try {
        const d = await getComponents();
        if (mounted) setData(d);
      } catch {
        if (mounted) setData(null);
      }
    }
    poll();
    const id = setInterval(poll, POLL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  if (!data?.available) {
    return (
      <Panel title="Components">
        <div className="text-dim text-sm">Shown with radio telemetry only (the GCS's link to the drone).</div>
      </Panel>
    );
  }

  const connected = data.components.filter((c) => c.status === "ok").length;
  return (
    <Panel title="Components">
      <div className="text-xs text-dim mb-1.5">
        {connected}/{data.components.length} connected
      </div>
      {data.components.map((c) => (
        <div key={c.key} className="py-1 border-b border-b-[#1d252d] last:border-b-0 text-[13px]">
          <div className="flex justify-between items-center gap-2">
            <span>{c.name}</span>
            <Pill kind={PILLS[c.status].kind}>{PILLS[c.status].label}</Pill>
          </div>
          {note(c) && <div className="text-dim text-xs tabular-nums">{note(c)}</div>}
        </div>
      ))}
    </Panel>
  );
}

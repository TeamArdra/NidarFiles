import { useEffect, useState } from "react";
import type React from "react";
import Panel from "./Panel";
import { getFlightTestStatus, getMissions, getRadioStatus, postAbort, postMissionStart } from "../api";
import type { FlightTestStatusResponse, Mission, RadioStatusResponse, TelemetryResponse } from "../types";

// The operator command surface is exactly START (for the selected mission)
// and ABORT -- see custom-gcs/CLAUDE.md Important Constraints #1. Choosing
// a mission in the dropdown is configuration, not a command. Do not add any
// other button that sends a command, and do not add a confirmation step
// before ABORT: abort must preempt everything immediately, per
// onboard-autonomy/CLAUDE.md Hard Safety Rule 2.
//
// Both commands go over the MicroLR900 command radio to the Jetson. What
// this panel shows is only what was actually confirmed:
//   1. sent over the radio / Jetson ACK (accepted or rejected with reason)
//   2. mission state from the Jetson (radio heartbeat, and the mission's status relayed over the radio)
//   3. vehicle state from telemetry (relayed over the radio)

const POLL_INTERVAL_MS = 1000;

// What START does, per mission -- shown under the buttons for the mission
// selected in the dropdown.
const MISSION_WARNINGS: Record<string, string> = {
  hover:
    "REAL FLIGHT. START (Hover) makes the Jetson put the vehicle in GUIDED, arm, take off, hover and land. " +
    "ABORT while airborne commands LAND. Keep the RC transmitter in hand as the independent override.",
  motor_test:
    "PROPS OFF. START (Motor Test) spins each motor in turn (A, B, C, D) at low throttle for a few seconds. " +
    "ABORT stops the motors immediately.",
  main_mission:
    "REAL FLIGHT. START (Main Mission) makes the Jetson take off, explore the arena, return to the entry and land. " +
    "ABORT while airborne lands where the drone is. Keep the RC transmitter in hand as the independent override.",
  main_mission_sim:
    "SIMULATOR. START (Main Mission (Sim)) flies ArduPilot SITL in the Gazebo maze. The real drone refuses this mission.",
};
const DEFAULT_WARNING = "REAL HARDWARE. START runs the selected mission on the vehicle. Keep the RC transmitter in hand.";

type Outcome = { ok: boolean; text: string } | null;

const RUNNING_STATES = new Set([
  "preflight", "setting_guided", "arming", "taking_off", "hovering", "exploring", "returning", "landing", "testing",
]);

function stateBadgeClass(state: string): string {
  if (state === "complete") return "bg-ok text-white";
  if (state === "failed") return "bg-bad text-white";
  if (state === "aborted" || state === "pilot_override") return "bg-warn text-black";
  if (RUNNING_STATES.has(state)) return "bg-[#1f6feb] text-white";
  return "bg-[#2b3a48] text-text"; // idle / unknown
}

function Badge({ className, children }: { className: string; children: React.ReactNode }) {
  return <span className={`inline-block rounded px-1.5 py-0.5 text-[11px] font-bold uppercase ${className}`}>{children}</span>;
}

function StatusLine({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="flex items-center gap-3 px-2.5 py-2 text-sm">
      <span className="w-20 shrink-0 text-xs text-dim">{label}</span>
      <div className="min-w-0 flex flex-wrap items-center">{children}</div>
    </div>
  );
}

function radioSummary(radio: RadioStatusResponse | null): { text: string; className: string } {
  if (!radio) return { text: "—", className: "text-dim" };
  if (!radio.enabled) return { text: "RADIO DISABLED", className: "text-bad font-semibold" };
  if (!radio.port_open) {
    return {
      text: `PORT ${radio.port ?? "?"} NOT OPEN${radio.port_error ? ` (${radio.port_error})` : ""}`,
      className: "text-bad font-semibold",
    };
  }
  if (!radio.jetson_link_up) return { text: "RADIO LINK DOWN — no Jetson heartbeat", className: "text-bad font-semibold" };
  return {
    text: `LINK UP · heartbeat ${radio.jetson_heartbeat_age_s ?? "?"} s ago`,
    className: "text-ok font-semibold",
  };
}

export default function ControlsPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const [missions, setMissions] = useState<Mission[] | null>(null);
  const [missionsError, setMissionsError] = useState<string | null>(null);
  const [selectedId, setSelectedId] = useState("");
  const [radio, setRadio] = useState<RadioStatusResponse | null>(null);
  const [mission, setMission] = useState<FlightTestStatusResponse | null>(null);
  // Independent busy/outcome state per button, deliberately -- a slow START
  // (radio retries) must never disable ABORT. See docs/DECISIONS.md D-10.
  const [startBusy, setStartBusy] = useState(false);
  const [abortBusy, setAbortBusy] = useState(false);
  const [startOutcome, setStartOutcome] = useState<Outcome>(null);
  const [abortOutcome, setAbortOutcome] = useState<Outcome>(null);

  useEffect(() => {
    let mounted = true;
    getMissions()
      .then((m) => {
        if (!mounted) return;
        setMissions(m);
        setSelectedId((prev) => prev || m[0]?.id || "");
      })
      .catch((e) => mounted && setMissionsError(e instanceof Error ? e.message : String(e)));
    return () => {
      mounted = false;
    };
  }, []);

  useEffect(() => {
    let mounted = true;
    async function poll() {
      const [r, f] = await Promise.allSettled([getRadioStatus(), getFlightTestStatus()]);
      if (!mounted) return;
      setRadio(r.status === "fulfilled" ? r.value : null);
      setMission(f.status === "fulfilled" ? f.value : null);
    }
    poll();
    const id = setInterval(poll, POLL_INTERVAL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  const selected = missions?.find((m) => m.id === selectedId) ?? null;

  async function start() {
    if (startBusy || !selected) return;
    setStartBusy(true);
    setStartOutcome({ ok: true, text: `Sending START (${selected.name}) over the radio…` });
    try {
      const res = await postMissionStart(selected.id);
      setStartOutcome({
        ok: true,
        text: `Jetson ACCEPTED START (${res.mission}) — ${res.attempts} radio attempt${res.attempts === 1 ? "" : "s"}`,
      });
    } catch (e) {
      setStartOutcome({ ok: false, text: `START not accepted: ${e instanceof Error ? e.message : String(e)}` });
    } finally {
      setStartBusy(false);
    }
  }

  async function abort() {
    if (abortBusy) return;
    setAbortBusy(true);
    setAbortOutcome({ ok: true, text: "Sending ABORT over the radio…" });
    try {
      const res = await postAbort();
      setAbortOutcome({ ok: true, text: `Jetson ACCEPTED ABORT — ${res.attempts} radio attempt${res.attempts === 1 ? "" : "s"}` });
    } catch (e) {
      setAbortOutcome({
        ok: false,
        text: `ABORT not confirmed: ${e instanceof Error ? e.message : String(e)} — USE THE RC TRANSMITTER / KILL SWITCH`,
      });
    } finally {
      setAbortBusy(false);
    }
  }

  const link = radioSummary(radio);
  const fcu = telemetry?.fcu;
  // One mission line: the Jetson's radio heartbeat says which mission and
  // its state; the relayed status adds the detail text.
  const missionId = radio?.jetson_link_up ? radio.jetson_mission ?? mission?.scenario ?? null : null;
  const missionState = radio?.jetson_link_up
    ? (radio.jetson_mission_state && radio.jetson_mission_state !== "unknown"
        ? radio.jetson_mission_state
        : mission?.state ?? null)
    : null;
  const missionName = missions?.find((m) => m.id === missionId)?.name ?? missionId ?? "";

  return (
    <Panel title="Mission Control">
      {missionsError && <div className="mb-2.5 text-bad text-xs font-semibold">{missionsError}</div>}

      <label className="block text-xs text-dim mb-3">
        Mission
        <select
          aria-label="Mission"
          className="mt-1 w-full bg-black/30 border border-border rounded px-2 py-1.5 text-sm text-text"
          value={selectedId}
          disabled={!missions || startBusy}
          onChange={(e) => setSelectedId(e.target.value)}
        >
          {(missions ?? []).map((m) => (
            <option key={m.id} value={m.id} className="bg-panel text-text">
              {m.name}
            </option>
          ))}
        </select>
      </label>

      <div className="flex gap-2.5 flex-wrap">
        <button
          type="button"
          className="font-semibold px-4 py-2.5 rounded-md border border-ok bg-ok text-white disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={startBusy || !selected}
          onClick={start}
        >
          START
        </button>
        <button
          type="button"
          className="font-semibold px-4 py-2.5 rounded-md border border-bad bg-bad text-white disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={abortBusy}
          onClick={abort}
        >
          STOP / ABORT
        </button>
      </div>

      {startOutcome && (
        <div className={`mt-2.5 text-xs font-semibold ${startOutcome.ok ? "text-ok" : "text-bad"}`}>{startOutcome.text}</div>
      )}
      {abortOutcome && (
        <div className={`mt-2.5 text-xs font-semibold ${abortOutcome.ok ? "text-ok" : "text-bad"}`}>{abortOutcome.text}</div>
      )}

      <div className="mt-3 rounded-md border border-border bg-black/20 divide-y divide-[#1d252d]">
        <StatusLine label="Radio link">
          <span className={link.className}>{link.text}</span>
        </StatusLine>
        <StatusLine label="Mission">
          {missionState ? (
            <>
              <span className="mr-2 text-text">{missionName}</span>
              <Badge className={stateBadgeClass(missionState)}>{missionState.replace(/_/g, " ")}</Badge>
            </>
          ) : (
            <span className="text-dim">no mission status</span>
          )}
        </StatusLine>
        {mission?.detail && (
          <div className="px-2.5 py-2 text-sm text-text" aria-label="Mission detail">
            {mission.detail}
          </div>
        )}
        <StatusLine label="Vehicle">
          {fcu?.connected ? (
            <>
              <Badge className={fcu.armed ? "bg-bad text-white" : "bg-[#2b3a48] text-text"}>
                {fcu.armed ? "ARMED" : "DISARMED"}
              </Badge>
              <span className="ml-2 text-text">{fcu.mode ?? "?"}</span>
              {mission?.current_altitude_m != null && (
                <span className="ml-2 text-dim">alt {mission.current_altitude_m} m</span>
              )}
            </>
          ) : (
            <span className="text-bad font-semibold">Pixhawk not connected (no FCU telemetry)</span>
          )}
        </StatusLine>
      </div>

      <div className="mt-2.5 px-2.5 py-2 bg-warn/10 border border-warn/40 rounded-md text-warn text-xs font-semibold">
        {(selected && MISSION_WARNINGS[selected.id]) ?? DEFAULT_WARNING}
      </div>
    </Panel>
  );
}

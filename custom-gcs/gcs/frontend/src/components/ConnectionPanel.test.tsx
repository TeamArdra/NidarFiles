import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import ConnectionPanel, { control } from "./ConnectionPanel";
import * as api from "../api";
import type { ComponentsResponse, TelemetryResponse } from "../types";

const PARTS: ComponentsResponse = {
  available: true,
  components: [
    { key: "jetson", name: "Radio link (GCS ↔ Jetson)", status: "ok", age_s: 0.3, detail: "mission idle" },
    { key: "fcu", name: "Pixhawk (flight controller)", status: "ok", age_s: 0.5, detail: "GUIDED, disarmed" },
    { key: "battery", name: "Battery monitor", status: "ok", age_s: 0.5, detail: "15.80 V" },
    { key: "position", name: "Position estimate", status: "ok", age_s: 0.1, detail: null },
    { key: "orientation", name: "IMU / attitude", status: "ok", age_s: 0.1, detail: null },
    { key: "lidar", name: "LiDAR (RPLIDAR A2)", status: "lost", age_s: 7.25, detail: null },
    { key: "map", name: "SLAM map (Cartographer)", status: "missing", age_s: null, detail: null },
  ],
};

beforeEach(() => {
  vi.spyOn(api, "getComponents").mockResolvedValue(PARTS);
});

afterEach(() => {
  vi.restoreAllMocks();
});

// This panel has its own state x event surface never previously covered
// by any test (App.test.tsx only ever renders it with `telemetry: null`
// or one fully-connected, fresh-heartbeat payload): the GCS<->Jetson link
// and the Jetson<->Pixhawk (FCU) link fail independently, and heartbeat
// staleness is a boundary condition (`> STALE_HEARTBEAT_S`, not `>=`).

function telemetryWith(overrides: Partial<TelemetryResponse>): TelemetryResponse {
  return {
    connected: true,
    mission_state: "idle",
    fcu: { connected: true, armed: false, guided: false, mode: "GUIDED", system_status: 3 },
    battery: { voltage: null, current: null, percentage: null },
    pose: { position: null },
    velocity: null,
    attitude: null,
    gps: null,
    statustext: [],
    heartbeat_age_s: 0.2,
    autonomy: { state: null, objective: null, target: null, next_action: null },
    sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
    mapping: {
      available: false, resolution_m: null, width_cells: null, height_cells: null,
      origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null,
    },
    navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
    ...overrides,
  };
}

describe("ConnectionPanel", () => {
  it("shows both links as UNKNOWN and 'no heartbeat received' before any telemetry arrives", () => {
    render(<ConnectionPanel telemetry={null} />);

    expect(screen.getAllByText("UNKNOWN")).toHaveLength(2);
    expect(screen.getByText("no heartbeat received")).toBeInTheDocument();
  });

  it("shows GCS<->Jetson CONNECTED independently of a disconnected FCU link", () => {
    render(
      <ConnectionPanel
        telemetry={telemetryWith({
          connected: true,
          fcu: { connected: false, armed: null, guided: null, mode: null, system_status: null },
        })}
      />,
    );

    expect(screen.getByText("CONNECTED")).toBeInTheDocument();
    expect(screen.getByText("DISCONNECTED")).toBeInTheDocument();
  });

  it("shows GCS<->Jetson DISCONNECTED independently of a connected FCU link", () => {
    render(
      <ConnectionPanel
        telemetry={telemetryWith({
          connected: false,
          fcu: { connected: true, armed: false, guided: false, mode: "GUIDED", system_status: 3 },
        })}
      />,
    );

    expect(screen.getByText("DISCONNECTED")).toBeInTheDocument();
    expect(screen.getByText("CONNECTED")).toBeInTheDocument();
  });

  it("does not flag the heartbeat as stale at exactly the 3s threshold", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ heartbeat_age_s: 3 })} />);

    const value = screen.getByText("3.0 s");
    expect(value.className).not.toMatch(/text-warn/);
  });

  it("flags the heartbeat as stale just above the 3s threshold", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ heartbeat_age_s: 3.1 })} />);

    const value = screen.getByText("3.1 s");
    expect(value.className).toMatch(/text-warn/);
  });

  it("treats a null heartbeat_age_s as stale even though connected is true", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ connected: true, heartbeat_age_s: null })} />);

    const value = screen.getByText("no heartbeat received");
    expect(value.className).toMatch(/text-warn/);
  });

  it("does not flag a fresh heartbeat as stale", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ heartbeat_age_s: 0.4 })} />);

    const value = screen.getByText("0.4 s");
    expect(value.className).not.toMatch(/text-warn/);
  });
});

describe("control", () => {
  const fcu = (mode: string | null, armed: boolean | null, connected = true) =>
    ({ connected, armed, guided: null, mode, system_status: 3 });

  it("names who is flying from the mode", () => {
    expect(control(fcu("GUIDED", true))).toEqual({ mode: "GUIDED", who: "Jetson (autonomous)", kind: "ok" });
    expect(control(fcu("STABILIZE", true))?.who).toBe("RC pilot");
    expect(control(fcu("ALT_HOLD", true))?.who).toBe("RC pilot");
    expect(control(fcu("LAND", true))?.who).toBe("Pixhawk (failsafe/landing)");
    expect(control(fcu("STABILIZE", false))?.who).toBe("disarmed");
  });

  it("is unknown without a Pixhawk", () => {
    expect(control(fcu("GUIDED", true, false))).toBeNull();
    expect(control(undefined)).toBeNull();
  });
});

describe("ConnectionPanel parts", () => {
  it("shows the mode, who flies, and every part compactly", async () => {
    render(
      <ConnectionPanel
        telemetry={telemetryWith({ fcu: { connected: true, armed: true, guided: true, mode: "ALT_HOLD", system_status: 4 } })}
      />,
    );
    expect(screen.getByText("ALT_HOLD")).toBeInTheDocument();
    expect(screen.getByText("RC pilot")).toBeInTheDocument();
    expect(await screen.findByLabelText("Battery: connected")).toBeInTheDocument();
    expect(screen.getByText("15.80 V")).toBeInTheDocument();
    expect(screen.getByLabelText("LiDAR: lost")).toBeInTheDocument();
    expect(screen.getByLabelText("SLAM map: not found")).toBeInTheDocument();
    expect(screen.queryByText("Radio link (GCS ↔ Jetson)")).not.toBeInTheDocument(); // the link rows cover it
  });

  it("says nothing arrived instead of listing parts while the link is down", async () => {
    render(<ConnectionPanel telemetry={telemetryWith({ connected: false })} />);
    expect(await screen.findByText("No data from the drone's parts yet.")).toBeInTheDocument();
  });
});

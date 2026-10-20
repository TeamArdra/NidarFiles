import { afterEach, describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import ComponentsPanel from "./ComponentsPanel";
import * as api from "../api";
import type { ComponentsResponse } from "../types";

afterEach(() => {
  vi.restoreAllMocks();
});

const DATA: ComponentsResponse = {
  available: true,
  components: [
    { key: "jetson", name: "Radio link (GCS ↔ Jetson)", status: "ok", age_s: 0.3, detail: "mission idle" },
    { key: "fcu", name: "Pixhawk (flight controller)", status: "ok", age_s: 0.5, detail: "STABILIZE, disarmed" },
    { key: "lidar", name: "LiDAR (RPLIDAR A2)", status: "lost", age_s: 7.25, detail: null },
    { key: "map", name: "SLAM map (Cartographer)", status: "missing", age_s: null, detail: null },
  ],
};

describe("ComponentsPanel", () => {
  it("lists every part with its state and a count", async () => {
    vi.spyOn(api, "getComponents").mockResolvedValue(DATA);
    render(<ComponentsPanel />);
    expect(await screen.findByText("2/4 connected")).toBeInTheDocument();
    expect(screen.getAllByText("CONNECTED")).toHaveLength(2);
    expect(screen.getByText("STABILIZE, disarmed")).toBeInTheDocument();
    expect(screen.getByText("LOST")).toBeInTheDocument();
    expect(screen.getByText("nothing for 7.3 s")).toBeInTheDocument();
    expect(screen.getByText("NOT FOUND")).toBeInTheDocument();
    expect(screen.getByText("never heard")).toBeInTheDocument();
  });

  it("explains itself off the radio path", async () => {
    vi.spyOn(api, "getComponents").mockResolvedValue({ available: false, components: [] });
    render(<ComponentsPanel />);
    expect(await screen.findByText(/radio telemetry only/)).toBeInTheDocument();
  });
});

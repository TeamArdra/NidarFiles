import { afterEach, describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import LidarPanel, { rays } from "./LidarPanel";
import * as api from "../api";
import type { LidarResponse } from "../types";

function scan(distances: Record<number, number>): LidarResponse {
  const d: (number | null)[] = Array(72).fill(null);
  for (const [i, v] of Object.entries(distances)) d[Number(i)] = v;
  return { available: true, age_s: 0.4, angle_offset_deg: 0, increment_deg: 5, min_cm: 15, max_cm: 1200, distances_cm: d };
}

afterEach(() => {
  vi.restoreAllMocks();
});

describe("LidarPanel", () => {
  it("turns sectors into rays, skipping sectors with no return", () => {
    expect(rays(scan({ 0: 120, 18: 340 }))).toEqual([
      { angleDeg: 0, distanceM: 1.2 },
      { angleDeg: 90, distanceM: 3.4 },
    ]);
  });

  it("draws the scan and names the nearest obstacle", async () => {
    vi.spyOn(api, "getLidar").mockResolvedValue(scan({ 0: 120, 1: 125, 18: 340, 54: 90 }));
    render(<LidarPanel />);
    expect(await screen.findByRole("img", { name: "LiDAR scan" })).toBeInTheDocument();
    expect(screen.getByText("nearest 0.90 m at 270°")).toBeInTheDocument();
    expect(screen.getByText("4/72 sectors × 5°")).toBeInTheDocument();
    expect(screen.getByText("updated 0.4 s ago")).toBeInTheDocument();
  });

  it("says how to get a scan when none is arriving", async () => {
    vi.spyOn(api, "getLidar").mockResolvedValue({ ...scan({}), available: false });
    render(<LidarPanel />);
    expect(await screen.findByText(/check the LiDAR is plugged into the Jetson/)).toBeInTheDocument();
  });
});

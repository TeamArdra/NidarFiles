import { act, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as api from "../api";
import AltitudePanel, { polyline, toSamples } from "./AltitudePanel";

afterEach(() => {
  vi.restoreAllMocks();
  vi.useRealTimers();
});

describe("toSamples / polyline", () => {
  it("measures height above the take-off point", () => {
    const s = toSamples([[10, 2.39, 0.4]], 1.89);
    expect(s[0].h).toBeCloseTo(0.5);
    expect(s[0].vz).toBe(0.4);
  });

  it("uses the raw height before the first arm", () => {
    expect(toSamples([[10, 1.89, null]], null)[0].h).toBe(1.89);
  });

  it("puts the newest sample at the right edge and drops old ones", () => {
    const pts = polyline(
      [
        { t: 0, h: 0, vz: 0 },
        { t: 100, h: 0, vz: 0 },
        { t: 160, h: 1, vz: 0 },
      ],
      160,
      1,
    ).split(" ");
    expect(pts).toHaveLength(2); // t=0 is older than 60 s
    expect(pts[1]).toBe("300.0,0.0"); // newest, top of a 0..1 m scale
  });
});

describe("AltitudePanel", () => {
  it("shows the live height and climb rate", async () => {
    vi.useFakeTimers();
    vi.spyOn(api, "getAltitude").mockResolvedValue({
      available: true,
      now: 100,
      ground_z: 1.0,
      samples: [
        [99.6, 1.2, 0.3],
        [99.8, 1.45, 0.35],
      ],
    });
    render(<AltitudePanel />);
    await act(async () => undefined); // first poll: sees the arm reference, refetches
    await act(async () => {
      vi.advanceTimersByTime(250);
    });
    expect(screen.getByTestId("alt-now").textContent).toBe("0.45 m");
    expect(screen.getByTestId("alt-vz").textContent).toContain("0.35 m/s");
    expect(screen.getByText("height above take-off")).toBeTruthy();
  });

  it("asks only for samples newer than the last one", async () => {
    vi.useFakeTimers();
    const spy = vi.spyOn(api, "getAltitude").mockResolvedValue({
      available: true,
      now: 100,
      ground_z: null,
      samples: [[99.8, 0.2, 0]],
    });
    render(<AltitudePanel />);
    await act(async () => undefined);
    await act(async () => {
      vi.advanceTimersByTime(250);
    });
    expect(spy).toHaveBeenNthCalledWith(1, null);
    expect(spy).toHaveBeenLastCalledWith(99.8);
  });
});

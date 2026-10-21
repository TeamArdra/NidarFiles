import { act, fireEvent, render, screen } from "@testing-library/react";
import { Subject } from "rxjs";
import { afterEach, describe, expect, it, vi } from "vitest";

const status$ = new Subject<boolean>();
const instances: Array<Record<string, ReturnType<typeof vi.fn>> & { config: unknown }> = [];

vi.mock("gzweb", () => ({
  SceneManager: class {
    config: unknown;
    destroy = vi.fn();
    resize = vi.fn();
    resetView = vi.fn();
    follow = vi.fn();
    thirdPersonFollow = vi.fn();
    getConnectionStatusAsObservable = () => status$.asObservable();
    constructor(config: unknown) {
      this.config = config;
      instances.push(this as never);
    }
  },
}));

import Sim3DPanel, { SIM_DRONE_ENTITY } from "./Sim3DPanel";

afterEach(() => {
  instances.length = 0;
});

describe("Sim3DPanel", () => {
  it("connects gzweb to the sim websocket and shows the status", async () => {
    render(<Sim3DPanel url="ws://127.0.0.1:9010" />);
    await act(async () => undefined); // dynamic import
    expect(instances).toHaveLength(1);
    expect(instances[0].config).toEqual({ elementId: "gz-sim-3d", websocketUrl: "ws://127.0.0.1:9010" });
    expect(screen.getByTestId("sim3d-status").textContent).toContain("connecting");
    act(() => status$.next(true));
    expect(screen.getByTestId("sim3d-status").textContent).toContain("connected");
  });

  it("chases the drone once connected and toggles to the free view", async () => {
    render(<Sim3DPanel url="ws://127.0.0.1:9010" />);
    await act(async () => undefined);
    act(() => status$.next(true));
    expect(instances[0].thirdPersonFollow).toHaveBeenCalledWith(SIM_DRONE_ENTITY);
    fireEvent.click(screen.getByRole("button", { name: "Free view" }));
    expect(instances[0].resetView).toHaveBeenCalled();
    expect(screen.getByRole("button", { name: "Follow drone" })).toBeTruthy();
  });

  it("releases the viewer when removed", async () => {
    const { unmount } = render(<Sim3DPanel url="ws://127.0.0.1:9010" />);
    await act(async () => undefined);
    unmount();
    expect(instances[0].destroy).toHaveBeenCalled();
  });
});

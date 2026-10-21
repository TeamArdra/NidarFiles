import { useEffect, useRef, useState } from "react";
import type { SceneManager } from "gzweb";
import Panel from "./Panel";

// Live 3D view of Main Mission (Sim): Gazebo's own scene (the maze, the
// simulated drone flying it) streamed over Gazebo's websocket server
// (scripts/sim/start_sim.sh) and drawn by Gazebo's browser viewer, gzweb.
// Shown ONLY when the backend reports a sim_view_url, i.e. when the GCS's
// "radio" is the simulator's TCP link -- never with the real MicroLR900, so
// the competition panel and its START/ABORT-only command surface are
// unchanged. The view buttons move the camera only; they send nothing to
// the drone.

const ELEMENT_ID = "gz-sim-3d";
// The drone's entity name in the sim world (sim/tools/make_sim_world.py).
export const SIM_DRONE_ENTITY = "nidar_drone";

type Status = "loading viewer" | "connecting" | "connected" | "disconnected" | string;

export default function Sim3DPanel({ url }: { url: string }) {
  const [status, setStatus] = useState<Status>("loading viewer");
  const [follow, setFollow] = useState(true);
  const scene = useRef<SceneManager | null>(null);

  useEffect(() => {
    let cancelled = false;
    let unsubscribe: (() => void) | undefined;
    // Loaded on demand: three.js + gzweb stay out of the normal panel's bundle.
    import("gzweb")
      .then(({ SceneManager }) => {
        if (cancelled) return;
        setStatus("connecting");
        const manager = new SceneManager({ elementId: ELEMENT_ID, websocketUrl: url });
        scene.current = manager;
        const sub = manager
          .getConnectionStatusAsObservable()
          .subscribe((ok) => setStatus(ok ? "connected" : "disconnected"));
        unsubscribe = () => sub.unsubscribe();
      })
      .catch((err) => !cancelled && setStatus(`3D viewer failed to load: ${String(err)}`));
    const onResize = () => scene.current?.resize();
    window.addEventListener("resize", onResize);
    return () => {
      cancelled = true;
      window.removeEventListener("resize", onResize);
      unsubscribe?.();
      scene.current?.destroy();
      scene.current = null;
    };
  }, [url]);

  // Camera: chase the drone, or the free orbit view. The drone's model can
  // arrive a moment after the connection, so the chase is re-applied once.
  useEffect(() => {
    if (status !== "connected" || !scene.current) return;
    const apply = () => (follow ? scene.current?.thirdPersonFollow(SIM_DRONE_ENTITY) : scene.current?.resetView());
    apply();
    const again = window.setTimeout(apply, 3000);
    return () => window.clearTimeout(again);
  }, [follow, status]);

  const dot = status === "connected" ? "bg-ok" : status === "connecting" || status === "loading viewer" ? "bg-warn" : "bg-bad";
  return (
    <Panel title="Simulation — Gazebo 3D (Main Mission (Sim))" className="col-span-full">
      <div className="flex items-center justify-between mb-2 text-xs">
        <span className="flex items-center gap-2" data-testid="sim3d-status">
          <span className={`inline-block w-2 h-2 rounded-full ${dot}`} />
          <span className="text-dim">Gazebo {url}:</span> <span>{status}</span>
        </span>
        <button
          type="button"
          className="px-2 py-0.5 rounded border border-border text-dim hover:text-text"
          onClick={() => setFollow((f) => !f)}
        >
          {follow ? "Free view" : "Follow drone"}
        </button>
      </div>
      <div id={ELEMENT_ID} className="w-full h-[440px] rounded-md overflow-hidden bg-black" />
      <div className="mt-1.5 text-[11px] text-dim">
        Simulator only (not shown with the real radio). Drag to orbit, scroll to zoom. The drone is the iris at the
        entry; START <b>Main Mission (Sim)</b> in Mission Control to fly it.
      </div>
    </Panel>
  );
}

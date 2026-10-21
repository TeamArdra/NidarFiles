"""Backend configuration. Pointing at the real Jetson instead of
sim/rosbridge_sim during development is a host/port change here, not a
code change anywhere else."""

from __future__ import annotations

import os
from dataclasses import dataclass


def _env_bool(name: str, default: bool) -> bool:
    """Parses a boolean-ish environment variable ("true"/"1"/"yes"/"on",
    case-insensitive, -> True; anything else present -> False; unset ->
    `default`). Never raises on a typo'd value -- it just falls through to
    False, which is always the more conservative reading for a switch
    like GCS_ROS_ENABLED below (accidentally disabling ROS is safe and
    obvious at a glance; accidentally enabling it on a typo is not)."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


TELEMETRY_SOURCES = ("radio", "rosbridge")


@dataclass(frozen=True)
class Settings:
    rosbridge_host: str = os.environ.get("ROSBRIDGE_HOST", "127.0.0.1")
    rosbridge_port: int = int(os.environ.get("ROSBRIDGE_PORT", "9090"))
    connect_timeout_s: float = float(os.environ.get("ROSBRIDGE_CONNECT_TIMEOUT_S", "5"))
    # Master switch for whether this backend attempts ROS/rosbridge
    # connectivity at all. Defaults to True -- the existing Jetson
    # deployment's behavior (always try to connect) is completely
    # unchanged unless this is explicitly opted out. Set
    # GCS_ROS_ENABLED=false for local frontend/backend development with
    # no Jetson, no ROS, no rosbridge available at all (e.g. a Windows
    # laptop) -- app/main.py then wires in app/ros_client.py's
    # DisabledRosBridgeClient instead of a real RosBridgeClient, so the
    # app never even attempts a roslibpy connection. Deliberately a
    # backend-only setting (GCS_ROS_ENABLED, not VITE_-prefixed): the
    # frontend has no ROS state of its own to hold, it only ever reflects
    # what this backend reports via GET /health's ros_status field -- see
    # app/schemas.py's HealthResponse.
    ros_enabled: bool = _env_bool("GCS_ROS_ENABLED", True)
    # Port of the Jetson's own lightweight MJPEG-over-HTTP server -- NOT
    # part of rosbridge (see docs/DECISIONS.md D-6, docs/DATA_MODELS.md
    # §7); this backend never proxies video bytes, only builds the URL.
    camera_stream_port: int = int(os.environ.get("CAMERA_STREAM_PORT", "8090"))
    # The MicroLR900 command radio (START/ABORT path -- see app/radio_link.py).
    # GCS_RADIO_PORT is the laptop's COM port for the radio (check Device
    # Manager > Ports); baud must match the radio's USB baud.
    radio_enabled: bool = _env_bool("GCS_RADIO_ENABLED", True)
    radio_port: str = os.environ.get("GCS_RADIO_PORT", "COM5")
    radio_baud: int = int(os.environ.get("GCS_RADIO_BAUD", "115200"))
    # Where telemetry comes from. "radio" (default): over the MicroLR900
    # radio from the Jetson's radio_command_node (app/radio_telemetry.py) --
    # there is no Wi-Fi link on the drone. "rosbridge": the old Wi-Fi path
    # (app/ros_client.py), still used with sim/rosbridge_sim for development.
    telemetry_source: str = os.environ.get("GCS_TELEMETRY_SOURCE", "radio").strip().lower()
    # Bench Setup page (read/write Pixhawk parameters over the radio). OFF by
    # default: the operator command surface in a mission is exactly START
    # and ABORT (CLAUDE.md Important Constraint 1), so the setup routes are
    # not even created unless this is set (start_gcs.ps1 -Setup).
    setup_enabled: bool = _env_bool("GCS_SETUP_ENABLED", False)
    # Main Mission (Sim): Gazebo's websocket (scripts/sim/start_sim.sh) for
    # the live 3D view panel. GCS_SIM_VIEW_URL wins; otherwise it is on
    # exactly when the "radio" is the simulator's TCP stand-in (socket://),
    # never with the real MicroLR900 -- the competition panel is unchanged.
    sim_view_url: str | None = os.environ.get("GCS_SIM_VIEW_URL") or None

    def sim_view(self) -> str | None:
        if self.sim_view_url:
            return self.sim_view_url
        if self.radio_port.startswith("socket://"):
            return "ws://127.0.0.1:9010"
        return None

    def __post_init__(self) -> None:
        if self.telemetry_source not in TELEMETRY_SOURCES:
            raise ValueError(
                f"GCS_TELEMETRY_SOURCE must be one of {TELEMETRY_SOURCES}, got {self.telemetry_source!r}"
            )

    def camera_stream_url(self) -> str:
        """Assumes the video server is co-located with the rosbridge host
        (i.e. the Jetson) -- a real deployment might need this decoupled
        into its own host setting later, out of scope for now."""
        return f"http://{self.rosbridge_host}:{self.camera_stream_port}/stream.mjpg"


def get_settings() -> Settings:
    return Settings()

#!/usr/bin/env bash
# Starts the GCS on a LINUX laptop (Windows: start_gcs.ps1). Everything goes
# over the MicroLR900 radio: START/ABORT out, telemetry back from the
# Jetson. There is no Wi-Fi link to the drone.
# Then open the operator panel at http://127.0.0.1:8000/ui/
#
# Usage:
#   scripts/gcs/start_gcs.sh                       # radio on /dev/ttyUSB0
#   scripts/gcs/start_gcs.sh --port /dev/ttyUSB1
#   scripts/gcs/start_gcs.sh --setup               # bench only: adds the
#                                                  # Pixhawk parameter Setup tab
#
# First run creates custom-gcs/gcs/backend/.venv and installs the backend
# requirements. The frontend is (re)built when its dist/ is missing or older
# than its sources (needs Node.js + npm).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BACKEND="$ROOT/custom-gcs/gcs/backend"
FRONTEND="$ROOT/custom-gcs/gcs/frontend"
PORT="/dev/ttyUSB0"
BAUD=115200
SETUP=false

while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --baud) BAUD="$2"; shift 2 ;;
    --setup) SETUP=true; shift ;;
    *) echo "unknown option: $1 (use --port DEV, --baud N, --setup)"; exit 1 ;;
  esac
done

# -- checks (warnings only: the GCS still starts and shows what is missing) --
if [[ "$PORT" == socket://* ]]; then
  echo "SIMULATOR: radio link over TCP $PORT (scripts/sim/start_sim.sh must be running)"
elif [ -e "$PORT" ]; then
  echo "OK: radio port $PORT present"
else
  echo "WARNING: radio port $PORT not found (ls /dev/ttyUSB* /dev/serial/by-id/). No telemetry and no START/ABORT until it is."
fi
if ! id -nG "$USER" | grep -qw dialout; then
  echo "WARNING: $USER is not in the 'dialout' group -- the radio port may be refused."
  echo "         Fix once: sudo usermod -aG dialout $USER   (then log out and back in)"
fi

# -- backend environment ------------------------------------------------------
if [ ! -x "$BACKEND/.venv/bin/python" ]; then
  echo "Creating backend virtualenv..."
  python3 -m venv "$BACKEND/.venv"
  "$BACKEND/.venv/bin/python" -m pip install -r "$BACKEND/requirements.txt"
fi

# -- frontend build -------------------------------------------------------------
BUILT="$FRONTEND/dist/index.html"
if [ ! -f "$BUILT" ] || [ -n "$(find "$FRONTEND/src" -type f -newer "$BUILT" -print -quit)" ]; then
  echo "Building frontend..."
  (cd "$FRONTEND" && { [ -d node_modules ] || npm install; } && npm run build)
fi

# -- run ----------------------------------------------------------------------
export GCS_TELEMETRY_SOURCE=radio
export GCS_ROS_ENABLED=false
export GCS_RADIO_PORT="$PORT"
export GCS_RADIO_BAUD="$BAUD"
export GCS_RADIO_ENABLED=true
export GCS_SETUP_ENABLED="$SETUP"
if $SETUP; then
  echo "WARNING: BENCH SETUP MODE: the panel can read/write Pixhawk parameters. Do not use this mode for a mission."
fi

echo "Starting GCS: radio $PORT @ $BAUD (commands + telemetry). Open http://127.0.0.1:8000/ui/"
(sleep 3 && command -v xdg-open >/dev/null && xdg-open "http://127.0.0.1:8000/ui/" >/dev/null 2>&1) &
cd "$BACKEND"
exec "$BACKEND/.venv/bin/python" -m uvicorn app.main:app --host 127.0.0.1 --port 8000

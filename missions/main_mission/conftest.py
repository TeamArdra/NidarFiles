"""Makes `nidar_autonomy` importable for these tests on a dev machine
(on the Jetson it is installed in ~/ros2_ws, so this is a no-op there)."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
try:
    import nidar_autonomy  # noqa: F401
except ImportError:
    sys.path.insert(0, os.path.join(_HERE, "..", "..", "onboard-autonomy", "nidar_autonomy"))

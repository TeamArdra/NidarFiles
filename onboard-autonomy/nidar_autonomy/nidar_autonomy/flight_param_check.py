"""Which Pixhawk parameters a flight needs in THIS setup -- no GPS,
optical flow, the Jetson as the only ground station, with or without an
RC safety pilot -- and whether the vehicle has them. Pure Python
(unit-tested); scripts/jetson/flight_params.py reads/writes the values
through MAVROS.

Every automatic failsafe ends in LAND:

  - Jetson / MAVROS goes silent -> LAND. ArduPilot's GCS failsafe watches
    heartbeats from system SYSID_MYGCS; MAVROS on the Jetson sends one
    every second as its own system id, so SYSID_MYGCS must equal MAVROS's
    system_id and FS_GCS_ENABLE = 5 (always LAND).
  - Battery low / critical -> LAND.
  - EKF (position estimate) failure -> LAND.

With an RC safety pilot (rc=True) the pilot takes over by flipping the
flight-mode switch to Stabilize / AltHold / Loiter; the mission sees the
mode leave GUIDED and stops commanding (PILOT_OVERRIDE). On top of the
above:

  - RC signal lost -> LAND (FS_THR_ENABLE = 3).
  - Jetson silent while the pilot is flying -> the pilot keeps control
    (FS_OPTIONS bit 16); in GUIDED it still lands.
  - The mode switch positions get Stabilize / AltHold / Loiter.

"apply" rules have a value the tool can set; "check" rules are only
reported (they depend on calibration or on choices made in Mission
Planner, e.g. ARMING_CHECK with the RC check unticked).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Mapping, Optional

FS_GCS_LAND = 5
FS_ACTION_LAND = 1
EKF_FS_LAND = 1
FS_THR_LAND = 3                 # RC failsafe: always LAND
FS_OPT_PILOT_KEEPS_CONTROL = 16  # FS_OPTIONS bit: continue in pilot-controlled modes on GCS failsafe
MODE_STABILIZE, MODE_ALTHOLD, MODE_LOITER = 0, 2, 5


@dataclass(frozen=True)
class Rule:
    name: str
    why: str
    target: Optional[float] = None  # value --apply sets (None: check only)
    ok: Optional[Callable[[float], bool]] = None  # custom test for check-only rules
    expected_text: str = ""
    fix_from: Optional[Callable[[float], float]] = None  # with ok: value --apply sets, from the current one


@dataclass(frozen=True)
class Result:
    name: str
    value: Optional[float]  # None = not read (unknown parameter / no reply)
    ok: bool
    expected: str
    why: str
    fix: Optional[float]  # value to set, or None if not auto-fixable


def _equal(a: float, b: float) -> bool:
    return abs(a - b) < 1e-3


def rules(mavros_system_id: int, rc: bool = False) -> List[Rule]:
    return [
        # -- Jetson-loss failsafe --
        Rule("SYSID_MYGCS", "Pixhawk treats the Jetson (MAVROS) as its ground station", float(mavros_system_id)),
        Rule("FS_GCS_ENABLE", "Jetson/MAVROS silent -> LAND", float(FS_GCS_LAND)),
        Rule("FS_GCS_TIMEOUT", "seconds of Jetson silence before LAND", 3.0),
        # -- other failsafes -> LAND --
        Rule("FS_EKF_ACTION", "position estimate fails -> LAND", float(EKF_FS_LAND)),
        Rule("BATT_FS_LOW_ACT", "battery low -> LAND", float(FS_ACTION_LAND)),
        Rule("BATT_FS_CRT_ACT", "battery critical -> LAND", float(FS_ACTION_LAND)),
        Rule("FS_THR_ENABLE", "RC signal lost -> LAND", float(FS_THR_LAND)) if rc
        else Rule("FS_THR_ENABLE", "no RC receiver: RC failsafe off", 0.0),
        *(_rc_rules() if rc else []),
        # -- battery limits for flight (4S) --
        Rule("BATT_ARM_VOLT", "no arming below 14.7 V (4S)", 14.7),
        Rule("BATT_LOW_VOLT", "low-battery failsafe at 14.5 V", 14.5),
        Rule("BATT_CRT_VOLT", "critical-battery failsafe at 14.0 V", 14.0),
        # -- logging --
        Rule("LOG_DISARMED", "log only while armed (SD card)", 0.0),
        # -- checked, not set: depend on setup choices --
        Rule("ARMING_CHECK",
             "pre-arm safety checks on (keep RC Channels ticked)" if rc
             else "pre-arm safety checks on (RC check may be unticked)",
             ok=lambda v: v != 0, expected_text="not 0"),
        Rule("FLOW_TYPE", "MTF-01 optical flow over MAVLink", ok=lambda v: _equal(v, 5), expected_text="5"),
        Rule("RNGFND1_TYPE", "MTF-01 rangefinder over MAVLink", ok=lambda v: _equal(v, 10), expected_text="10"),
        Rule("RNGFND1_MAX_CM", "rangefinder max range in cm (8 m)", ok=lambda v: v >= 400, expected_text=">= 400"),
        Rule("RNGFND1_ORIENT", "rangefinder pointing down", ok=lambda v: _equal(v, 25), expected_text="25"),
        Rule("EK3_SRC1_POSXY", "no GPS position", ok=lambda v: _equal(v, 0), expected_text="0"),
        Rule("EK3_SRC1_VELXY", "horizontal velocity from optical flow", ok=lambda v: _equal(v, 5), expected_text="5"),
        Rule("EK3_SRC1_YAW", "heading from compass", ok=lambda v: _equal(v, 1), expected_text="1"),
        Rule("EK3_SRC_OPTIONS", "ArduPilot flow guide setting", ok=lambda v: _equal(v, 0), expected_text="0"),
        Rule("COMPASS_USE", "compass in use (and calibrated!)", ok=lambda v: _equal(v, 1), expected_text="1"),
        Rule("GPS1_TYPE", "no GPS fitted", ok=lambda v: _equal(v, 0), expected_text="0"),
        Rule("GPS2_TYPE", "no GPS fitted", ok=lambda v: _equal(v, 0), expected_text="0"),
    ]


def _rc_rules() -> List[Rule]:
    """Extra rules when an RC safety pilot is on board."""
    return [
        # Jetson dies while the pilot flies -> pilot keeps control (GUIDED still lands)
        Rule("FS_OPTIONS", "Jetson silent while the pilot flies -> pilot keeps control",
             ok=lambda v: (int(v) & FS_OPT_PILOT_KEEPS_CONTROL) != 0, expected_text="bit 16",
             fix_from=lambda v: float(int(v) | FS_OPT_PILOT_KEEPS_CONTROL)),
        # 3-position mode switch: low / middle / high
        Rule("FLTMODE1", "mode switch low = Stabilize (pilot flies)", float(MODE_STABILIZE)),
        Rule("FLTMODE4", "mode switch middle = AltHold", float(MODE_ALTHOLD)),
        Rule("FLTMODE6", "mode switch high = Loiter (holds position on optical flow)", float(MODE_LOITER)),
        Rule("FLTMODE_CH", "RC channel of the mode switch (match the transmitter)",
             ok=lambda v: 5 <= v <= 16, expected_text="5-16"),
        Rule("RC3_MIN", "RC calibrated in Mission Planner (throttle low end)",
             ok=lambda v: 800 <= v <= 1200, expected_text="800-1200"),
    ]


def names(mavros_system_id: int = 1, rc: bool = False) -> List[str]:
    return [r.name for r in rules(mavros_system_id, rc)]


def evaluate(values: Mapping[str, Optional[float]], mavros_system_id: int, rc: bool = False) -> List[Result]:
    results = []
    for rule in rules(mavros_system_id, rc):
        value = values.get(rule.name)
        if rule.target is not None:
            expected = f"{rule.target:g}"
            ok = value is not None and _equal(value, rule.target)
            fix = None if ok or value is None else rule.target
        else:
            expected = rule.expected_text
            ok = value is not None and bool(rule.ok(value))
            fix = rule.fix_from(value) if rule.fix_from and not ok and value is not None else None
        results.append(Result(rule.name, value, ok, expected, rule.why, fix))
    return results


def report(results: List[Result], rc: bool = False) -> str:
    lines = []
    for r in results:
        if r.ok:
            tag = "OK     "
        elif r.value is None:
            tag = "UNKNOWN"
        elif r.fix is not None:
            tag = "FIX    "
        else:
            tag = "CHECK  "
        value = "?" if r.value is None else f"{r.value:g}"
        lines.append(f"{tag} {r.name:<16} = {value:<8} (want {r.expected:<6}) {r.why}")
    bad = [r for r in results if not r.ok]
    fixable = [r for r in bad if r.fix is not None]
    lines.append("")
    if not bad:
        lines.append("ALL OK: flight parameters are set for a flight with an RC safety pilot." if rc
                     else "ALL OK: flight parameters are set for a no-RC flight.")
    else:
        lines.append(
            f"{len(bad)} not OK; {len(fixable)} can be set with --apply (disarmed). "
            "CHECK/UNKNOWN items need Mission Planner or the sensor setup."
        )
    return "\n".join(lines)

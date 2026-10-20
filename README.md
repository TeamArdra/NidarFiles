# NidarFiles — NIDAR AirMouse: radio START → real hover

> **Status (2026-10-08).** Radio-only (no Wi-Fi). Proven on the real
> hardware, props off: radio START/ABORT, telemetry over the radio, ARM via
> radio START, the full Hover sequence (GUIDED → ARM → TAKEOFF → LAND →
> disarm), the Motor Test mission, and the EKF origin set by the Jetson.
> **Not flown yet.** Flights are planned **without an RC transmitter**:
> every failsafe ends in LAND, including the Jetson going silent. Before the
> first flight: calibrations, flow orientation check, flight parameters,
> charged battery, tether — see [11. Pre-flight checklist](#11-pre-flight-checklist-first-hover).

New here? Read sections 1–5 in order, then 6–7 each session.

**Contents:** 1 What this is · 2 How it fits together · 3 What you need ·
4 First-time setup · 5 Every session: start-up and checks · 6 What dry-run
does · 7 Missions · 8 Setup tab (Pixhawk parameters) · 9 Updating the code ·
10 Safety · 11 Pre-flight checklist · 12 Troubleshooting · 13 Pixhawk
configuration · 14 For developers · 15 Test status · 16 Where we stopped

---

## 1. What this is

NIDAR AirMouse is an autonomous indoor (no GPS) search-and-rescue drone. This
repository makes its first real mission work: **press START on the laptop and
the drone takes off, hovers, and lands.**

| Part | What it does |
|---|---|
| **GCS laptop** (Windows or Linux) | The operator panel in a web browser. Shows the drone's state and has exactly two commands: **START** (the mission chosen in the dropdown) and **STOP / ABORT**. |
| **2 × MicroLR900 radios (900 MHz)** | The **only** link between the laptop and the drone: commands out, telemetry back. There is **no Wi-Fi**. |
| **Jetson Orin Nano** (on the drone, Ubuntu Linux + ROS 2) | The mission computer: receives the radio command, checks it, runs the mission, talks to the Pixhawk. |
| **Pixhawk 6X** (ArduCopter 4.6.3) | The flight controller: actually flies the drone. Connected to the Jetson by Ethernet. |
| **MicoAir MTF-01** (on the Pixhawk's TELEM1) | Optical flow + downward rangefinder: the drone's indoor position source. |

Missions in the dropdown: **Hover** (take off to 0.5 m, hold 10 s, land) and
**Motor Test** (props off: spin each motor in turn).

## 2. How it fits together

```
COMMANDS (START / ABORT)
Laptop ──USB── MicroLR900 )))) 900 MHz )))) MicroLR900 ──USB── Jetson ──Ethernet── Pixhawk 6X
               radio_command_node checks the command ─▶ mission node ─▶ MAVROS ─▶ Pixhawk

TELEMETRY (same radio, other direction)
Pixhawk ─▶ MAVROS ─▶ Jetson radio_command_node ─▶ radio ─▶ laptop GCS ─▶ operator panel
   flight mode, armed, battery, position, attitude, Pixhawk messages, mission progress
```

- The radio ends at the **Jetson**, not the Pixhawk. The Jetson commands the
  Pixhawk through **MAVROS** over Ethernet.
- Every START is checked on the Jetson first (known mission, mission
  program running, Pixhawk connected, nothing else running). The panel shows
  **"Jetson ACCEPTED"** or **"REJECTED: <reason>"** — it never claims success
  without the Jetson's answer.
- **ABORT** is always accepted: on the ground → disarm; in the air → **LAND**
  (never a mid-air motor stop).
- Not available without Wi-Fi: live video, map and perception panels (the
  radio is too slow). The missions here don't use them.

## 3. What you need

**Hardware:** the drone (Pixhawk + Jetson + MTF-01 + radio, all mounted),
the second MicroLR900 radio for the laptop, a charged 4S battery, a monitor
+ keyboard for the Jetson, a USB cable for the Pixhawk (Mission Planner), a
**tether** (rope to a heavy weight) or net, and a textured, well-lit mat for
take-off. No RC transmitter is used (see 10. Safety).

**Software on the laptop**

| | Windows | Linux (Ubuntu/Debian) |
|---|---|---|
| Git | `winget install Git.Git` | `sudo apt install git` |
| Python 3.10+ | `winget install Python.Python.3.12` | `sudo apt install python3 python3-venv python3-pip` |
| Node.js 18+ (builds the panel) | `winget install OpenJS.NodeJS.LTS` | Node 18+ from [nodejs.org](https://nodejs.org) or `nvm`; check `node --version` |
| Radio USB driver | Silicon Labs **CP210x** driver if no COM port appears | built in |
| Mission Planner (Pixhawk setup/calibration) | [ardupilot.org](https://ardupilot.org/planner/) | (use a Windows PC, or QGroundControl) |

**Software on the Jetson:** Ubuntu 22.04 with ROS 2 Humble (JetPack).
`setup_jetson.sh` installs the rest (MAVROS, pyserial, colcon) if missing —
that step needs internet once.

## 4. First-time setup

### 4.1 Laptop: get the code

Pick the terminal you use; each block is complete on its own.

**Windows — Command Prompt (CMD)**
```bat
cd /d D:\
git clone https://github.com/kp00004/NidarFiles.git
cd /d D:\NidarFiles
```

**Windows — PowerShell**
```powershell
Set-Location D:\
git clone https://github.com/kp00004/NidarFiles.git
Set-Location D:\NidarFiles
```

**Linux laptop — terminal**
```bash
cd ~
git clone https://github.com/kp00004/NidarFiles.git
cd ~/NidarFiles
sudo usermod -aG dialout $USER     # once: permission for the radio's USB port; then log out and back in
```

The first GCS start (section 5.3) creates the Python environment and builds
the panel automatically (a few minutes).

### 4.2 Jetson: get the code and build (monitor + keyboard, internet once)

```bash
git clone https://github.com/kp00004/NidarFiles.git ~/NidarFiles
~/NidarFiles/scripts/jetson/setup_jetson.sh
```

Run `setup_jetson.sh` as your normal user — **not with `sudo`** (it asks for
your password itself where needed). It:
1. installs missing packages (MAVROS, pyserial, colcon),
2. adds you to `dialout` (permission for the radio) — **log out and back in** if it says so,
3. offers to disable **ModemManager** — answer **`y`** (it grabs USB radios),
4. builds the Jetson code into `~/nidar_ws`.

It ends with `Summary: 2 packages finished`.

### 4.3 Pixhawk: one-time configuration

Done once with Mission Planner over USB — see [13. Pixhawk configuration](#13-pixhawk-configuration).
Already done on this drone except the calibrations in section 11.

### 4.4 Radios

Both MicroLR900 radios must match: 115200 baud, DUPLEX, HIGH rate, MAX
power, address 1000, channel 0 (already set on this pair).

## 5. Every session: start-up and checks

### 5.1 Hardware checklist (before power-on)

- [ ] **Props OFF** for anything on the bench (they go on only for a planned flight).
- [ ] Battery charged (4S: ≥ 15.2 V; full = 16.8 V) and connected.
- [ ] Jetson ↔ Pixhawk Ethernet cable connected.
- [ ] Radio #1 in the Jetson's USB, radio #2 in the laptop's USB, antennas on.
- [ ] MTF-01 lens clean and unobstructed; drone on a **textured, well-lit** surface (plain/shiny floors and shadow break optical flow).
- [ ] Drone **still** while the Pixhawk powers up (it calibrates its gyros then).

### 5.2 Jetson: start the stack (one terminal)

```bash
~/NidarFiles/scripts/jetson/start_jetson.sh             # LIVE, EVERYTHING: START really runs the mission; LiDAR (7.3) + live map (7.4)
~/NidarFiles/scripts/jetson/start_jetson.sh --dry-run   # same, but commands are answered and never executed
~/NidarFiles/scripts/jetson/start_jetson.sh --setup     # LIVE + allows parameter writes from the Setup tab
~/NidarFiles/scripts/jetson/start_jetson.sh --no-map    # no SLAM map (LiDAR panel still on)
~/NidarFiles/scripts/jetson/start_jetson.sh --no-lidar  # no LiDAR, so no map either
~/NidarFiles/scripts/jetson/start_jetson.sh --no-fcu    # NO Pixhawk connected: radio link + LiDAR + map only (START refused)
```

A LiDAR that isn't plugged in, or Cartographer not installed, is only a
`WARNING` — the rest of the stack starts. Map and LiDAR packets share the
radio with START/ABORT: if ACKs start needing retries, use `--no-map`.

Without a Pixhawk the normal start stops at `Pixhawk … does not answer` and
the radio never comes up — use `--no-fcu` for radio/LiDAR work on the bench.

Options can be combined (`--dry-run --setup`). **Ctrl+C** stops everything.
Logs: `~/NidarFiles/logs/<date_time>/` (one file per program).

What you should see, in order:

| Line | Meaning |
|---|---|
| `Pixhawk reachable at 192.168.144.14` | Ethernet link OK (the script sets the Jetson's address if missing) |
| `MAVROS connected` | Jetson ↔ Pixhawk talking |
| `started command_node … radio_command_node … hover_mission … motor_test_mission` | all programs running |
| `radio serial open: /dev/serial/by-id/usb-Silicon_Labs_CP2102…` | Jetson radio found |
| `stream 32 @ 20.0 Hz: ok` (×4) | Pixhawk sends position/attitude/battery fast enough |
| `setting EKF origin: lat 12.916500 lon 79.132500` then `EKF origin confirmed by FCU` | the Pixhawk knows where "zero" is (needed for GUIDED take-off indoors) |
| `DRY RUN: …` or `LIVE: …` | which mode you started |

Read-only diagnostics at any time (second terminal):
```bash
~/NidarFiles/scripts/jetson/check_jetson.sh
```
Look for `OK:` lines: radio port, `dialout`, ModemManager inactive, `eno1` address, Pixhawk ping, MAVROS state, **local position publishing**, **EKF origin set**, battery voltage — and at the end the **flight parameter check** (next).

**Flight parameters** (second terminal, stack running):
```bash
~/NidarFiles/scripts/jetson/flight_params.py            # check: OK / FIX / CHECK per parameter
~/NidarFiles/scripts/jetson/flight_params.py --apply    # set all FIX items (refused while armed), then reboot the Pixhawk
```
It checks everything a no-RC flight needs: Jetson silent → LAND
(`SYSID_MYGCS`, `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=3`), battery/EKF
failsafes → LAND, flight battery limits (14.7/14.5/14.0 V), `LOG_DISARMED=0`,
and the optical-flow/EKF setup. **CHECK** items it won't change (e.g.
`ARMING_CHECK`, sensor settings) — fix those in Mission Planner. It ends with
`ALL OK` when ready.

### 5.3 Laptop: start the GCS

**Windows — PowerShell**
```powershell
Set-Location D:\NidarFiles
powershell -ExecutionPolicy Bypass -File .\scripts\gcs\start_gcs.ps1
```

**Windows — Command Prompt (CMD)**
```bat
cd /d D:\NidarFiles
powershell -ExecutionPolicy Bypass -File scripts\gcs\start_gcs.ps1
```

**Linux laptop**
```bash
cd ~/NidarFiles
./scripts/gcs/start_gcs.sh
```

Options: radio port — Windows `-RadioPort COM7` (find it in Device Manager →
Ports), Linux `--port /dev/ttyUSB1` (`ls /dev/ttyUSB*`); Setup tab —
Windows `-Setup`, Linux `--setup`.

The browser opens **http://127.0.0.1:8000/ui/**. Keep the terminal open
(closing it stops the GCS).

### 5.4 Checks in the panel before any START

| Where | Expected | If not |
|---|---|---|
| **Components** | every part `CONNECTED`: radio link, Pixhawk, battery monitor, position estimate, IMU, LiDAR, SLAM map (`LOST` = it stopped, `NOT FOUND` = never heard) | the row names the part to check; LiDAR/map are `NOT FOUND` with `--no-lidar`/`--no-map` |
| Mission Control → **Radio link** | `LINK UP · heartbeat 0.x s ago` | section 12: RADIO LINK DOWN / PORT NOT OPEN |
| Mission Control → **Mission** | mission name + `IDLE` | Jetson stack not running, or mission program crashed (its log) |
| Mission Control → **Vehicle** | `DISARMED` + flight mode | `Pixhawk not connected`: MAVROS/Ethernet (check_jetson.sh) |
| **Battery** | voltage above `BATT_ARM_VOLT` (14.7 V for flight) | charge the battery |
| **Position / Velocity** | numbers, changing when the drone is carried | no indoor position: optical flow/EKF (section 12) |
| **Messages** panel | no red `CRITICAL`/`ERROR` lines like `PreArm: …` | each has a plain-English hint under it |
| Footer | `telemetry: MicroLR900 command radio` | GCS started the wrong way |

## 6. What dry-run does

`start_jetson.sh --dry-run` is the **safe test mode**:

- Everything starts as normal: MAVROS, the radio, telemetry, the missions.
- A START/ABORT from the laptop **is received, checked and answered** — the
  panel shows `Jetson ACCEPTED START` (or the rejection reason) exactly as in
  live mode, and the Jetson logs `DRY RUN: not publishing 'start'`.
- But the command is **never passed on to the missions**, so **nothing can
  arm, spin or fly**. The mission stays `idle`.

Use it to test the radio link, the panel and the START checks with no risk.
Use live mode (no `--dry-run`) only when you mean the mission to run.

## 7. Missions

Every mission step also appears in the **Messages** panel, labelled with the
mission name (e.g. `Hover: armed -- taking off to 0.50 m`), next to the
Pixhawk's own messages. The Pixhawk's messages explain refusals (e.g.
`PreArm: Need Position Estimate`).

### 7.1 Hover (real flight)

1. Checks in 5.4 pass. **Mission: Hover** → **START**.
2. The panel shows `Jetson ACCEPTED START (hover)`, then the mission goes through these steps (Mission line + Messages panel):

| State | Message |
|---|---|
| `preflight` → `setting_guided` | `Hover: requesting GUIDED` |
| `arming` | `Hover: GUIDED confirmed -- arming` |
| `taking_off` | `Hover: armed -- taking off to 0.50 m` |
| `hovering` | `Hover: hovering at 0.50 m for 10s` |
| `landing` | `Hover: hover complete -- landing` |
| `complete` | `Hover: landed and disarmed -- hover complete` |

3. **ABORT** any time: before arming → stops; while arming → disarms; in the air → LAND.

Automatic stops (each lands or disarms, and shows the reason): preflight
problems (`Hover: preflight failed: …` — e.g. no position, no EKF origin,
already armed), take-off not acknowledged, target altitude not reached in
15 s, **more than 0.5 m above target** (e.g. lifted by hand), drifting more
than 0.75 m sideways, position lost, Pixhawk link lost. If the safety pilot
switches flight mode on the RC transmitter, the mission stops commanding at
once (`pilot has control`).

Settings: `CONFIG` at the top of `missions/hover/mission.py` (take-off
height, hold time, limits). The EKF origin (Vellore) is `EKF_ORIGIN` there.

### 7.2 Motor Test (PROPS OFF)

Checks every motor runs, in the right order and direction. Uses ArduCopter's
own motor test (the same as Mission Planner's Motor Test page) — no position
estimate needed.

1. **Props off.** Jetson started **live**. **Mission: Motor Test** → **START**.
2. Motors spin one at a time at 8 % for 5 s, 1 s apart: **A** (front-right),
   **B** (back-right), **C** (back-left), **D** (front-left). Messages:
   `Motor Test: motor A (1/4) at 8% for 5 s` … then
   `Motor Test: all 4 motors tested -- check order and direction`, plus the
   Pixhawk's `starting motor test` / `finished motor test`.
3. Check each against the ArduPilot Quad-X diagram: right position; A and C
   spin counter-clockwise, B and D clockwise.
4. **ABORT** stops the running motor immediately. Each motor also stops by
   itself after 5 s even if the radio or Jetson fails.

ArduCopter refuses (message shown) if the vehicle is armed, the safety switch
isn't pressed, or the RC isn't calibrated. Settings: `CONFIG` in
`missions/motor_test/mission.py`.

The Jetson refuses a START for one mission while the other is running
(`MISSION_BUSY`).

### 7.3 LiDAR live view (RPLIDAR A2, over the radio)

Not a mission — a live display. Plug the RPLIDAR A2 into a Jetson USB port;
`start_jetson.sh` runs it by default (`--no-lidar` to leave it off).

- The LiDAR's USB adapter and the radio use the same chip (CP2102). Their
  Linux names differ on our units, but such adapters *may* share a name, so
  the script doesn't rely on names: whenever two or more USB serial ports
  are plugged in (even with `--no-lidar`), `start_jetson.sh` asks each
  port which one is the RPLIDAR and gives the radio the other one. It prints
  `RPLIDAR on /dev/ttyUSBx …` and `radio -> /dev/ttyUSBy`. Both work at the
  same time. Verified on the real A2M12 (model 0x2C, 256000 baud, ~12 scans/s).
- `lidar_node` publishes the full scan on ROS `/scan` (for SLAM later);
  the radio carries a **72-sector summary** (nearest obstacle every 5°,
  ~180 bytes) **once per second**.
- The GCS **LiDAR** panel shows it as a radar view: drone in the middle,
  **top = the drone's nose**, range rings, the nearest obstacle in amber.

Settings (environment variables before `start_jetson.sh`):
`NIDAR_LIDAR_RATE_HZ` (default 1; 2 for a smoother view, if START/ABORT
answers stay on the 1st attempt), `NIDAR_LIDAR_YAW_DEG` — the direction of
the LiDAR's 0° mark (its motor/cable side faces backwards on the A2)
relative to the nose, clockwise (whole numbers are fine, e.g. `270`). Check it: put a box in front of the drone,
it must appear at the top of the panel; if it appears at the right, set 270
(…at the bottom 180, at the left 90). If no LiDAR answers, the stack starts
without it (`WARNING: no RPLIDAR answered`).

### 7.4 Live map (Cartographer SLAM, over the radio)

```bash
~/NidarFiles/scripts/jetson/start_jetson.sh --no-fcu --dry-run   # hand-held, no Pixhawk
~/NidarFiles/scripts/jetson/start_jetson.sh                      # on the drone
```

The map is on by default (`--no-map` to leave it off): the LiDAR plus **Cartographer** 2D SLAM.
**With the Pixhawk connected** (no `--no-fcu`), the script asks it for the
IMU at 50 Hz, measures `/mavros/imu/data` for 8 s, and if it gets ≥ 30 Hz
Cartographer also uses the **Pixhawk IMU** (levels tilted scans, keeps up with
fast turns -- needed in flight). Otherwise, or with `--no-fcu`, it's **LiDAR
only** (fine hand-held). The start-up log says which: `Cartographer SLAM
(LiDAR + Pixhawk IMU)` or `(LiDAR only)`. The IMU only helps if the Pixhawk
moves **with** the LiDAR -- Pixhawk on the bench while you carry the LiDAR:
`NIDAR_MAP_IMU=off`. Cartographer builds the
map and works out where the LiDAR is; the Jetson shrinks the map to
**0.25 m cells** (wall / free / unknown) and sends only changed rows over the
radio (~100–300 B/s), plus the LiDAR's position and heading. The GCS **Map**
panel shows it, north up, with a blue dot + heading line for the LiDAR,
"updated … s ago" and the position in metres.

Hand-held mapping: carry the LiDAR, Jetson and its radio together (battery
/ power bank), **keep the LiDAR level**, walk slowly (~0.5 m/s), turn
slowly. Rooms with furniture, corners and doorways map best; long plain
corridors and glass are hard. The map starts where the LiDAR is switched on
(that spot is 0,0).

Needs Cartographer once: `setup_jetson.sh` installs
`ros-humble-cartographer-ros` (internet). Settings: `NIDAR_MAP_RATE_HZ`
(map packets per second, default 2), `NIDAR_MAP_CELL_M` (default 0.25; the
competition grid is 1 m), `NIDAR_LIDAR_MIN_RANGE_M` (default 0.4 m: closer
returns are ignored -- the person holding the LiDAR, or the drone's own
frame/props; raise it if a white blob follows the blue dot). With the IMU:
`NIDAR_MAP_IMU` (`auto` / `off`), `NIDAR_MAP_IMU_MIN_HZ` (default 30), and
the LiDAR's position from the Pixhawk `NIDAR_LIDAR_X_M` / `_Y_M` / `_Z_M`
(metres, x forward, y left, z up; default 0). `NIDAR_LIDAR_YAW_DEG` must be
right: the IMU and the scan have to agree on which way is forward. If the map
never appears with the IMU, run once with `NIDAR_MAP_IMU=off` to confirm the
LiDAR side works, then check `ros2 topic echo --once /mavros/imu/data`
(frame_id should be `base_link`). Cartographer's log: `cartographer.log`.

Reading the map: dark = free space seen through, white = walls/obstacles,
grey = not seen yet. Walls fill in as you walk along them; a wall seen
from one spot only is dotted.

**1 m arena grid** (what the judges score: walls/openings per 1 m cell, and
survivor grid boxes). Every 3 s the Jetson lines the SLAM map up with the
arena's 1 m grid by itself -- it finds the wall directions and where the
grid lines fall, assuming thin walls standing on the 1 m lines -- and sends
every cell edge over the radio (`ARENA_GRID`, ~145 B/s). On the Map panel:
**orange = wall, green = open, faint grey = not seen properly yet**; the
blue-tinted square is cell (0, 0), where the LiDAR started (stand it inside
the first cell, roughly facing along a wall), and the dashed blue square is
the cell it's in now. The line above the map gives the grid size, wall /
open / unknown counts and the current cell. An edge only shows open once
free space is seen on **both** sides of it. The Jetson log says
`arena grid aligned: … deg` the first time it locks on. Off:
`arena_rate_hz:=0` on `radio_command_node`.

## 8. Setup tab (Pixhawk parameters)

For bench work only — **never during a mission**. Start both sides in setup mode:

- Jetson: `start_jetson.sh --setup`
- Laptop: Windows `… start_gcs.ps1 -Setup`, Linux `./scripts/gcs/start_gcs.sh --setup`

Two tabs appear at the top: **Mission** and **Setup: Pixhawk parameters**.
In Setup: type a parameter name (or click one of the quick buttons), **Read**,
change the value, **Write**. The value shown is always read back from the
Pixhawk. The Jetson refuses writes while armed, while a mission runs, or
without `--setup` — the reason is shown. Some parameters need a Pixhawk
reboot to take effect. Calibrations (compass, accelerometer) still need
Mission Planner, because the drone must be turned by hand.

Without setup mode there are no tabs and no way to change parameters from the
panel (the competition allows the operator only START and ABORT).

## 9. Updating the code

When new code is on GitHub:

| Where | Command |
|---|---|
| Jetson | `git -C ~/NidarFiles pull` then `~/NidarFiles/scripts/jetson/setup_jetson.sh` (rebuild; harmless if nothing changed) |
| Windows CMD | `cd /d D:\NidarFiles` then `git pull` |
| Windows PowerShell | `Set-Location D:\NidarFiles; git pull` |
| Linux laptop | `git -C ~/NidarFiles pull` |

The GCS rebuilds the panel by itself at the next start. Restart both sides
after updating.

No internet on the Jetson: `scripts/gcs/make_jetson_bundle.ps1 -Destination E:\`
(Windows) copies the Jetson files to a USB stick; copy `NidarFiles` from the
stick to `~/` on the Jetson and run `setup_jetson.sh`.

## 10. Safety

There is **no RC transmitter**, so nobody can take over in the air. The
only commands are **ABORT (= LAND)** over the radio and, on the ground,
pulling the battery. That is why:

- **Props off** for everything on the bench. In live mode START (Hover) arms and flies; START (Motor Test) spins the motors.
- Every flight is **tethered** (short rope to a heavy weight, so it can't climb above ~1 m or drift far) or inside a net. People stay behind the drone and out of reach. One person on the laptop, finger on **ABORT**.
- **Low and short:** 0.5 m for 10 s. The mission lands by itself if it climbs above 1.0 m, drifts more than 0.75 m, loses position, or can't reach the height in 15 s.
- **Every failsafe ends in LAND** (set by `flight_params.py --apply`):
  - **Jetson or MAVROS goes silent** (crash, power, cable) → the Pixhawk lands after 3 s on its own (`FS_GCS_ENABLE=5`, heartbeat from the Jetson's MAVROS = `SYSID_MYGCS`).
  - Battery low/critical → LAND. Position estimate (EKF) fails → LAND.
- **Radio lost:** the mission still finishes its hover and lands; ABORT is unavailable until the link returns.
- **Optical flow orientation must be checked before flying** (section 11). A wrongly oriented flow sensor makes the drone accelerate away instead of holding still — with no RC there is no recovery except the tether.
- The mission follows any LAND the Pixhawk starts by itself and never overrides it.
- Don't run `mission_state_node` with this stack (it would arm on START by itself); the scripts refuse.

### 10.1 With an RC safety pilot (recommended)

A transmitter + receiver (on the Pixhawk's RC IN) lets a pilot take over:
**flip the mode switch** and the drone is theirs. The sticks alone do
nothing in GUIDED (ArduCopter ignores roll/pitch/throttle there) — the
**switch** is the take-over. The mission sees the mode leave GUIDED,
reports `pilot_override` on the GCS and stops commanding; it never
switches back. Set up with `flight_params.py --rc --apply` (section 11):

| Switch | Mode | What the pilot gets |
|---|---|---|
| low (`FLTMODE1`) | **Stabilize** | full manual: sticks = attitude, **throttle stick = throttle** |
| middle (`FLTMODE4`) | AltHold | holds height; sticks move it |
| high (`FLTMODE6`) | Loiter | holds position on optical flow; sticks move it |

- **Throttle stick at the bottom for START** (ArduCopter refuses to arm with the throttle up). **Once airborne, move it to about the middle and keep it there** — GUIDED ignores it, so this is harmless.
- **Before flipping to Stabilize, the throttle stick must be near hover (~middle).** Stabilize takes the throttle straight from the stick: a stick at the bottom drops the drone. AltHold/Loiter are the gentler take-over (centre stick = hold).
- **RC signal lost → LAND** (`FS_THR_ENABLE=3`).
- **Jetson dies while the pilot is flying → the pilot keeps control** (`FS_OPTIONS` bit 16). In GUIDED (no pilot) it still lands after 3 s.
- Keep "RC Channels" **ticked** in `ARMING_CHECK` and calibrate the radio in Mission Planner (Setup → Mandatory Hardware → Radio Calibration). Transmitter **on before** powering the drone.
- Strongly suggested: a **motor emergency stop** switch on a spare channel (`RCx_OPTION=31`, e.g. `RC7_OPTION=31`).
- Competition note: the operator may only START/abort — any manual input costs **−50**. The RC is a safety backup, not a way to fly the mission.

## 11. Pre-flight checklist (first hover)

All with the drone **fully assembled** (Jetson, wiring and battery mounted
change the magnetic field — calibrate after assembly, never before).
**Props off until step 10.**

| # | Step | How / pass when |
|---|---|---|
| 1 | Power-up check | power up **still**, wait 2 min; `start_jetson.sh --dry-run` + GCS: LINK UP, Pixhawk connected, position shown, battery ≥ 15.2 V |
| 2 | Accelerometer calibration | Mission Planner (USB) → Setup → Mandatory Hardware → Accel Calibration, 6 positions |
| 3 | Compass calibration | … → Compass → Start, rotate in every direction away from metal; `Check mag field` gone after reboot |
| 4 | RC (section 10.1) | **With RC:** Radio Calibration, mode switch on `FLTMODE_CH`, check each switch position shows Stabilize / AltHold / Loiter in Mission Planner (after step 7). **Without RC:** if Messages shows an `RC …` PreArm, `ARMING_CHECK` → untick **only "RC Channels"** |
| 5 | **Optical flow direction** | carry the drone ~1 m **forward**, then **sideways**: Position x/y must change the matching way and stop when you stop; if reversed/swapped set `FLOW_ORIENT_YAW` (e.g. 18000 if mounted rotated 180°) and repeat |
| 5b | **Altitude check** | note Position **z** with the drone on the floor; lift it (level, still) to **0.5 m** and **1.0 m** above the floor (tape measure) for ~10 s each: z must rise by about that much (±0.15 m) and stay steady. Jumpy or off by > 0.3 m → baro is too noisy indoors: set `EK3_SRC1_POSZ=2` (rangefinder height) and repeat |
| 6 | Motor Test | Jetson live; section 7.2: order A-B-C-D, A/C counter-clockwise, B/D clockwise |
| 7 | Flight parameters | `flight_params.py --apply` (**`--rc --apply`** with an RC pilot), reboot the Pixhawk, run the same check again → `ALL OK` (CHECK items fixed in Mission Planner) |
| 8 | Bench Hover START | Jetson live, props off: GUIDED → ARM → TAKEOFF accepted, then `landed and disarmed (target altitude not reached…)` — expected without props |
| 9 | **Jetson-loss failsafe test** | props off: Hover START; while `taking_off`, press **Ctrl+C** on the Jetson stack. Within ~3 s the Pixhawk must switch to **LAND** and disarm (watch in Mission Planner over USB). Restart the stack afterwards |
| 9b | **RC take-over test** (with RC) | props off: Hover START; while `taking_off` flip the switch to Stabilize → GCS shows `pilot_override`, Mission Planner shows STABILIZE, the Jetson sends nothing more. Then switch off the transmitter → mode LAND (RC failsafe). Disarm |
| 10 | First hover | **props on, tethered**, people clear, take-off mat lit; Jetson live; Hover START, finger on ABORT (and the pilot on the switch, throttle near middle) |

## 12. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `PORT COM5 NOT OPEN` (Linux: `/dev/ttyUSB0`) | radio unplugged or another port: Device Manager → Ports / `ls /dev/ttyUSB*`; start with `-RadioPort` / `--port`; close other programs using it |
| `RADIO LINK DOWN` | Jetson stack not running; Jetson radio missing (`check_jetson.sh`); radios not paired |
| `START not accepted … never acknowledged` | same as RADIO LINK DOWN |
| `REJECTED: MISSION_NOT_READY` | mission program not running/crashed — see `hover_mission.log` / `motor_test_mission.log` |
| `REJECTED: FCU_NOT_CONNECTED` | MAVROS not connected: Ethernet cable, `ping 192.168.144.14`, `mavros.log` |
| `REJECTED: MISSION_BUSY` | a mission is still running — wait or ABORT |
| `Pixhawk not connected (no FCU telemetry)` | MAVROS lost the Pixhawk (rebooted? cable?) — `check_jetson.sh` |
| `PreArm: Need Position Estimate` | optical flow not usable: light + texture under the drone; `RNGFND1_MAX_CM` must be 800 (not 8) |
| EKF `started relative aiding` / `stopped aiding` repeating | same as above |
| `PreArm: Check mag field` / `mag anomaly` | compass not calibrated or metal nearby — calibrate (section 11) |
| `PreArm: Compass not healthy` | compass priority points at a missing compass — Mission Planner → Compass → Remove Missing |
| `PreArm: Gyros inconsistent` | drone moved during power-up or still warming — reboot it still, wait 2 min; accel calibration |
| `Battery … failsafe` / `below minimum arming voltage` | charge; the failsafe latches until the Pixhawk reboots |
| `Hover: preflight failed: EKF origin not set` | hover log: was `EKF origin confirmed` printed? MAVROS connected? |
| Setup tab: `writes disabled` | Jetson not started with `--setup` |
| Jetson: `Permission denied` on a script | `chmod +x ~/NidarFiles/scripts/jetson/*.sh` |
| Jetson: radio permission denied | not in `dialout`: rerun `setup_jetson.sh`, log out/in |
| `WARNING: no RPLIDAR answered` | LiDAR USB unplugged or no power (it needs USB 5 V; the motor should spin when the stack starts); check `ls /dev/ttyUSB*` shows two ports |
| LiDAR panel: obstacles in the wrong direction | set `NIDAR_LIDAR_YAW_DEG` (section 7.3) |
| Radio link drops (START/ABORT need retries) | lower `NIDAR_LIDAR_RATE_HZ` (e.g. 0.5) / `NIDAR_MAP_RATE_HZ`, or start with `--no-map` |
| Windows: script "cannot be loaded … disabled on this system" | use `powershell -ExecutionPolicy Bypass -File …` as shown |
| `PreArm: RC not calibrated` / `RC not found` / `Throttle below failsafe` | no RC in this setup: `ARMING_CHECK` → untick only "RC Channels"; `FS_THR_ENABLE=0` |
| `flight_params.py`: `UNKNOWN` items | MAVROS hasn't loaded the parameter list yet — wait ~30 s after `MAVROS connected` and run again |
| Pixhawk didn't LAND when the Jetson stopped (step 9) | `SYSID_MYGCS` must equal MAVROS `system_id` (the script prints it); `FS_GCS_ENABLE=5`; Pixhawk rebooted after setting |

## 13. Pixhawk configuration

Current settings on this drone (ArduCopter 4.6.3):

| Group | Parameters |
|---|---|
| MTF-01 on TELEM1 | `SERIAL1_PROTOCOL=1`, `SERIAL1_BAUD=115`, `SERIAL1_OPTIONS=1024`; sensor set to `Mav_APM`, `mav_id 200` via MicoAssistant (ArduPilot serial passthrough: `SERIAL_PASS2=1`, `SERIAL_PASSTIMO=120`, then back to `-1`) |
| Optical flow / rangefinder | `FLOW_TYPE=5`, `RNGFND1_TYPE=10`, `RNGFND1_ORIENT=25`, `RNGFND1_MIN_CM=1`, `RNGFND1_MAX_CM=800`, `RNGFND1_GNDCLEAR=25` (cm, sensor height when landed) |
| EKF (no GPS) | `EK3_SRC1_POSXY=0`, `EK3_SRC1_VELXY=5`, `EK3_SRC1_POSZ=1`, `EK3_SRC1_VELZ=0`, `EK3_SRC1_YAW=1`, `EK3_SRC_OPTIONS=0`; origin set by the Jetson |
| GPS | `GPS1_TYPE=0`, `GPS2_TYPE=0` (no GPS fitted) |
| Compass | built-in BMM150 (ID 331777) as priority 1, `COMPASS_USE=1` — **needs calibration** |
| Checks / logging | `ARMING_CHECK=1` (untick "RC Channels" if no RC complains); `LOG_DISARMED=1` while bench testing → 0 for flight |
| Battery (bench values) | `BATT_ARM_VOLT=13`, `BATT_LOW_VOLT=13.2`, `BATT_CRT_VOLT=12.8` — flight: 14.7 / 14.5 / 14.0 (`flight_params.py --apply`) |
| Failsafes for flight (no RC) | `SYSID_MYGCS` = MAVROS system_id (1), `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=3`, `FS_EKF_ACTION=1`, `BATT_FS_LOW_ACT=1`, `BATT_FS_CRT_ACT=1`, `FS_THR_ENABLE=0` — all set by `flight_params.py --apply` |

Network: Jetson `eno1` 192.168.144.1/24 ↔ Pixhawk 192.168.144.14 (MAVLink
UDP 14550). The Jetson address is set by `start_jetson.sh` each time (not
saved). Radio on the Jetson: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.

## 14. For developers

```
NidarFiles/
├── custom-gcs/          GCS: FastAPI backend (gcs/backend) + React panel (gcs/frontend), docs/
├── onboard-autonomy/    Jetson ROS 2 packages (nidar_autonomy: radio node, codec, vehicle layer)
├── missions/            hover/ and motor_test/: mission.py (ROS node) + *_logic.py (pure, tested)
└── scripts/             jetson/: setup, start, check · gcs/: start_gcs.ps1 / .sh, USB bundle
```

Key files: radio protocol `onboard-autonomy/nidar_autonomy/nidar_autonomy/telem_command_codec.py`
(mirrored in `custom-gcs/gcs/backend/app/radio_protocol.py`), START rules
`radio_command_logic.py`, telemetry `radio_telemetry.py` (both sides),
parameter rules `param_bridge_logic.py`. Interface docs:
`custom-gcs/docs/COMMUNICATION.md` §6.

Run the tests (laptop, from the repo root; backend venv created by the GCS start script):

| | Windows (PowerShell) | Linux |
|---|---|---|
| Python env | `custom-gcs\gcs\backend\.venv\Scripts\python.exe -m pip install pytest` | `custom-gcs/gcs/backend/.venv/bin/pip install pytest` |
| Missions | `custom-gcs\gcs\backend\.venv\Scripts\python.exe -m pytest missions -q` | `custom-gcs/gcs/backend/.venv/bin/python -m pytest missions -q` |
| Jetson code | `cd onboard-autonomy\nidar_autonomy; ..\..\custom-gcs\gcs\backend\.venv\Scripts\python.exe -m pytest test -q` | `cd onboard-autonomy/nidar_autonomy && ../../custom-gcs/gcs/backend/.venv/bin/python -m pytest test -q` |
| GCS backend | `cd custom-gcs\gcs\backend; .venv\Scripts\python.exe -m pytest -q` | `cd custom-gcs/gcs/backend && .venv/bin/python -m pytest -q` |
| GCS panel | `cd custom-gcs\gcs\frontend; npx tsc --noEmit; npx vitest run` | same |
| Jetson start script (float parameters) | `custom-gcs\gcs\backend\.venv\Scripts\python.exe -m pytest scripts -q` (needs Git Bash) | `custom-gcs/gcs/backend/.venv/bin/python -m pytest scripts -q` |

## 15. Test status

| What | Result |
|---|---|
| Unit tests: missions 56, Jetson 415, GCS backend 180, panel 112 | PASS |
| Radio protocol vs pymavlink; Jetson ↔ GCS in-memory (commands, telemetry, parameters) | PASS |
| Radio link on hardware: START/ABORT ACK on 1st try, telemetry live, unplug/replug recovery | **PASS** (2026-10-07) |
| ARM via radio START; full Hover sequence to `landed and disarmed` (props off) | **PASS** (2026-10-08) |
| Motor Test mission on the motors | **PASS** (ran; order/direction to confirm) |
| EKF origin set by the Jetson | **PASS** (`EKF3 IMU0 origin set`) |
| Hover mission "too high" safety (drone lifted by hand) → LAND → disarm | **PASS** |
| Setup tab (parameters over the radio) on hardware | NOT TESTED |
| `flight_params.py` and the Jetson-loss → LAND failsafe on hardware | NOT TESTED (checklist steps 7 and 9) |
| Flight | NOT TESTED |

## 16. Where we stopped (2026-10-08, evening)

- Optical flow fixed (`RNGFND1_MAX_CM` was 8 cm); EKF holds a position on a lit, textured surface.
- Flights will be **without RC**. Added: the Jetson-loss → LAND failsafe and
  `scripts/jetson/flight_params.py` (checks/sets every flight parameter).
- The Jetson was on the bench (cables out → panel said "Pixhawk not
  connected", expected). **Next session: everything mounted on the drone, then
  the checklist in section 11 from step 1.**
- Code workflow (maintainers): edit in `D:\NidarFiles`, copy to
  `D:\NidarFiles-github`, push to https://github.com/kp00004/NidarFiles. The
  `custom-gcs` / `onboard-autonomy` folders inside `D:\NidarFiles` are also the
  TeamArdra repos (branch `feature/hover-radio`); this work isn't committed there.

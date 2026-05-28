# CIWS

**C**lose-**I**n **W**eapon **S**ystem — a Raspberry Pi 4 based mobile platform combining LiDAR navigation, YOLO-based person detection, and a pan/tilt nerf turret with auto/manual targeting modes.

---

## Overview

CIWS is built around a tank-style chassis with the following capabilities:

- **Autonomous navigation** using a 360° RPLidar with obstacle avoidance
- **Manual driving** via WASD keyboard control
- **Person detection** using YOLOv8 running on a USB webcam
- **Pan/tilt turret** with two servos (X and Z axes)
- **Trigger firing** via a relay-controlled mechanism
- **Auto mode** that aims the turret at detected people and fires when within range
- **Manual mode** for hands-on turret aiming and firing
- **Watchdog failsafe** that engages safe state if any subsystem goes silent
- **Live LiDAR visualization** in a side-by-side window with the camera feed

---

## Hardware

| Component | Notes |
|---|---|
| Raspberry Pi 4 | Running Raspberry Pi OS |
| Slamtec RPLidar | 360° scanning LiDAR (CP2102 USB-serial) — **must be plugged into a USB 2.0 (black) port**, not USB 3.0 (blue) |
| USB Webcam | Default 640×480 resolution |
| L298N motor driver | Controls two DC tread motors |
| Two servos | SG90/MG90 class — X-axis (pan) and Z-axis (tilt) |
| Relay module | Drives the nerf trigger mechanism |

### GPIO pinout (BCM numbering)

| Function | Pin |
|---|---|
| Motor IN1 | D26 |
| Motor IN2 | D13 |
| Motor IN3 | D21 |
| Motor IN4 | D27 |
| Motor PWM A | D19 |
| Motor PWM B | D4 |
| Servo X (pan) | D17 |
| Servo Z (tilt) | D18 |
| Trigger relay | D24 |

---

## Software stack

- Python 3.11 inside the `CVsys` virtual environment (`virtualenvwrapper`)
- OpenCV (Qt backend on Linux)
- Ultralytics YOLO (v8n weights)
- Adafruit Blinka (board, digitalio, pwmio)
- adafruit-circuitpython-rplidar
- adafruit-motor

---

## Quick start

```bash
ciws
```

That's it. The `ciws` alias (defined in `~/.bashrc`) activates the `CVsys` venv and runs `CIWS_Combined.py`.

If the alias isn't set up, the equivalent is:

```bash
workon CVsys
cd /home/CIWS
python3 CIWS_Combined.py
```

### Startup sequence

The program prompts for three choices in order:

1. **Navigation mode:** `[A]utonomous` / `[M]anual` / `[O]ff`
2. **Speed profile** (if A or M): `[S]low` / `[N]ormal` / `[F]ast`
3. **Trigger mode:** `[A]uto` / `[M]anual`

After all three are answered, the camera and LiDAR windows open and the system goes live.

---

## Controls

### Quitting

`q` at any time fully shuts down the program. This works from:

- The terminal (caught by the keyboard dispatcher)
- The camera window (caught by OpenCV)
- The LiDAR window (caught by OpenCV)

`Ctrl+C` also triggers the same clean shutdown.

### Navigation (manual mode only)

| Key | Action |
|---|---|
| `w` | Forward |
| `s` | Reverse |
| `a` | Turn left |
| `d` | Turn right |
| (release) | Stop |

### Trigger (manual mode only)

| Key | Action |
|---|---|
| `i` | Tilt up |
| `k` | Tilt down |
| `j` | Pan left |
| `l` | Pan right |
| `f` | Fire |

### Camera window

| Key | Action |
|---|---|
| `f` | Toggle fullscreen |
| `q` | Shut down CIWS |

---

## Configuration

### `nav_config.json`

Located at `/home/CIWS/nav_config.json`. Edit anytime — changes take effect on next startup, no code changes needed.

```json
{
    "speeds": {
        "slow":   { "forward": 30, "turn": 30, "backward": 30 },
        "normal": { "forward": 50, "turn": 45, "backward": 40 },
        "fast":   { "forward": 100, "turn": 75, "backward": 75 }
    },
    "thresholds": {
        "turn_mm": 500,
        "reverse_mm": 200
    },
    "front_sector_degrees": 30,
    "scan_hz": 10
}
```

| Field | What it does |
|---|---|
| `speeds.*.forward` | Forward PWM duty cycle (%) for that profile |
| `speeds.*.turn` | Turning PWM duty cycle (%) |
| `speeds.*.backward` | Reverse PWM duty cycle (%) |
| `thresholds.turn_mm` | If anything in the front sector is closer than this, robot turns away |
| `thresholds.reverse_mm` | If anything in the front sector is closer than this, robot reverses (must be less than `turn_mm`) |
| `front_sector_degrees` | Width of the forward-facing "obstacle detection cone" |
| `scan_hz` | Target navigation loop rate |

### Other tunables (in code)

Some values still live in source. Worth knowing where to find them:

- `FIRE_AREA_THRESHOLD` (in `CIWS_Combined.py`, `aim_from_detection`): how big a detected person must be in pixels² before auto-firing. Default 100,000.
- `COOLDOWN_SECONDS` (same place): minimum delay between auto-fire shots. Default 3.
- `TURN_COMMIT_CYCLES` (in `navigation.py`, `_run_autonomous`): how many scan cycles the robot sticks with a turn direction before re-evaluating. Default 5. Increase for less wiggle, decrease for more responsive turns.
- `MAX_CONSECUTIVE_FAILURES` (same place): after this many failed LiDAR scans in a row, motors stop for safety. Default 10.
- `stale_threshold` (in `CIWS_Combined.py`, `CIWSControl.__init__`): seconds before the watchdog flags a subsystem as stale. Default 3.0.

---

## Architecture

### File layout

```
/home/CIWS/
├── CIWS_Combined.py          # Main entry point — wires everything together
├── KeyboardDispatcher.py     # Threaded keyboard listener
├── nav_config.json           # Speed profiles + thresholds
├── run_ciws.sh               # Launcher (activates venv, runs main)
├── README.md                 # This file
├── Sensors/
│   ├── navigation.py         # LiDAR + motors + autonomous logic
│   └── lidar_visualizer.py   # Top-down point cloud window
└── Actuators/
    └── TriggerControl.py     # Servo aim + relay trigger + safe state
```

### Threads at runtime

```
Main thread
├── PersonDetectorThread.run         (also draws LiDAR visualizer window)
├── init_sys → spawns:
│   ├── KeyboardDispatcher           (terminal keystrokes)
│   ├── _autonomous_thread           (in NavigationSystem, if autonomous)
│   └── aim_thread                   (if trigger in auto mode)
└── _watchdog_loop                   (monitors heartbeats)
```

### Data flow

```
Camera frame → YOLO → DetectionBuffer ──┐
                                        ├── aim_from_detection → TriggerControl.aim/shoot
LiDAR scan → ScanBuffer ────────────────┤
                                        ├── _run_autonomous → MotorController
                                        └── LidarVisualizer (drawn each YOLO frame)

All subsystems → Watchdog.beat() → _watchdog_loop → TriggerControl.enter_safe_state()
```

### Key design decisions

- **In-memory data sharing.** Detection data lives in a `DetectionBuffer` (thread-safe dict) rather than a JSON file on disk. Same for LiDAR scans (`ScanBuffer`). No disk I/O in the hot loop.
- **Single GUI thread.** Both the camera and LiDAR windows are drawn from the detector thread. This sidesteps GTK/Qt's hard requirement that GUI calls happen from one consistent thread.
- **Anti-oscillation in navigation.** Once the robot picks a turn direction, it commits for 5 scan cycles before reconsidering. Prevents the "stuck wiggling between left and right" failure mode.
- **Last-good-scan fallback.** If a LiDAR scan fails, the navigation logic uses the previous good scan rather than zeros. Zeros look like "all clear" to the avoidance logic, which would cause the robot to drive into obstacles after a single failed scan.
- **Watchdog failsafe.** Every subsystem heartbeats once per loop iteration. If any goes silent for more than 3 seconds, motors stop and trigger enters safe state (aim ignored, fire blocked, servos centered). Recovers automatically when heartbeats resume.

---

## Troubleshooting

### `RPLidar port not found`

The OS doesn't see the LiDAR on any serial port.

1. Unplug the LiDAR USB, wait 10 seconds, plug back into a **USB 2.0 (black) port**.
2. Run `ls /dev/ttyUSB*` — you should see `/dev/ttyUSB0`. If not:
3. Run `dmesg | tail -20` and look for `cp210x converter now attached`. If you see attach followed by an immediate disconnect, it's a USB power issue → try a powered USB hub.
4. Verify you're in the `dialout` group: `groups`. If not: `sudo usermod -aG dialout $USER`, then log out and back in.

### `Incorrect descriptor starting bytes` / `New scan flags mismatch`

LiDAR serial protocol is out of sync — usually from a previous unclean shutdown that left the motor spinning.

1. `q` or `Ctrl+C` to exit cleanly.
2. If that doesn't fix it, unplug LiDAR USB for 10 seconds and reconnect.

### `'CIWSControl' object has no attribute 'X'`

A method (likely `stop`) got pulled out of the class by an editor stripping its indent. Open `CIWS_Combined.py` and make sure the method is indented 4 spaces (so it's inside the class). The body should be at 8 spaces.

To find what's at the wrong level:

```bash
grep -n "^def " /home/CIWS/CIWS_Combined.py
```

Any `def` flush-left that should be a class method shows up here.

### Camera window opens but LiDAR window doesn't

The visualizer is now drawn from the camera thread (see "Key design decisions"). If only the camera window appears:

1. Confirm the LiDAR initialized successfully — look for `[Nav] RPLidar detected on /dev/ttyUSB0` in the startup output.
2. If navigation is in `Off` mode, no scans run, so the LiDAR window shows no points but should still appear.

### Aim slews to one corner instead of following the person

Camera resolution and trigger aim mapping are out of sync. The detector should print `[Camera] Resolution: 640x480` and the trigger should print `[Trigger] Aim calibrated for 640x480` (or whatever your resolution actually is). If they don't match, edit 5 in the "real camera resolution" change didn't land — check `TriggerControl.__init__` accepts `frame_width` and `frame_height` parameters.

### Watchdog keeps engaging safe state during normal operation

YOLO is running too slowly on the Pi and the camera heartbeat is missing the 3-second threshold. Two options:

- Increase `stale_threshold` in `CIWSControl.__init__` (e.g. from 3.0 to 5.0)
- Use a smaller YOLO model (`yolov8n.pt` is already the smallest, so this is rarely an option)

### Qt warnings on shutdown

`QT_LOGGING_RULES=*=false` is set at the top of `CIWS_Combined.py` to silence them. If they reappear, confirm that line is still present and runs *before* any OpenCV imports.

### Terminal is broken after a crash

The keyboard dispatcher puts the terminal in `cbreak` mode. If the program crashes before restoring it, the terminal becomes weird (no echo, etc.). To recover:

```bash
reset
```

(Type it blind, hit Enter.)

---

## Operating notes

### Order of operations on startup

The system blocks on three sequential prompts before any threads start full operation. This is intentional — it lets you configure the run before the camera, motors, or servos start moving. If you `Ctrl+C` during a prompt, the program may exit before all subsystems are initialized.

### Auto-fire behavior

In auto trigger mode, the system fires when:

1. A person is detected
2. The bounding box area exceeds `FIRE_AREA_THRESHOLD` (100,000 px²)
3. At least `COOLDOWN_SECONDS` (3 s) have passed since the last shot
4. The trigger is NOT in safe state

The watchdog can engage safe state at any time, instantly blocking auto-fire.

### LiDAR visualization

- Red dot = the robot, at the center
- Red arrow = forward heading
- Yellow dots = LiDAR returns within 3 meters
- Grey rings = 1m / 2m / 3m range markers
- Green wedge = the configured front sector (where obstacle avoidance is checking)
- Top-left text = age of the latest scan (should stay under ~150 ms during normal operation)

### Shutdown sequence

A clean exit produces this output in order:

```
[CIWS] Shutting down...
[Camera] Detection stopped.
[Nav] Shutting down navigation...
[Nav] Autonomous loop ended; motors stopped.
[Nav] Navigation shutdown complete.
[Trigger] Cleanup complete.
[Watchdog] Stopped.
[CIWS] All systems shutdown.
```

If you see `Aborted` or `terminate called without an active exception` after `All systems shutdown`, something C-level (probably the LiDAR or OpenCV) didn't clean up. Usually harmless for the current run but can leave the LiDAR in a stuck state for the next run.

---

## Known limitations / future work

- **No persistent mapping.** LiDAR scans are used for reactive obstacle avoidance only. No SLAM, no path planning.
- **Aim mapping assumes camera FOV matches servo range.** A person at the edge of the camera frame triggers a near-extreme servo angle (0° or 180°), which may not match the turret's actual aim envelope. Calibration scaling factor would help.
- **No fire suppression when servos are saturated.** If `aim()` clamps to 0° or 180°, the turret is at its mechanical stop, not pointing at the target. Firing in that state is guaranteed to miss. (Listed as a planned enhancement.)
- **Subsystem startup prompts are sequential.** All three could be read once at startup, or come from a config file. Lower priority since it's only annoying during dev iteration.
- **No logging to file.** All output is print statements to stdout. A real logging setup (timestamps, levels, file output) would make post-incident debugging much easier.

---

## Credits

- Original CIWS framework: Alfonso S. et al.
- LiDAR & navigation: Alfonso S.
- Vision / detection: Ultralytics YOLOv8
- Refactoring + documentation: collaborative development with Claude


# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DOGZILLA is a 12-DOF quadruped robot (by Yahboom Technology) powered by Raspberry Pi 5. It combines hardware servo control via serial, a Flask-based web control app, and a ROS 2 (Humble) workspace for autonomous navigation and perception.

## Build & Run Commands

### DOGZILLALib (install once)
```bash
cd ~/dogzilla/DOGZILLALib
sudo python3 setup.py install
# or: pip install -e .
```

### ROS 2 Workspace (`yahboomcar_ws`)
```bash
# Build entire workspace
cd ~/dogzilla/yahboomcar_ws
colcon build

# Build a single package
colcon build --packages-select <package_name>

# Source after building
source install/setup.bash

# Launch the base robot (cmd_vel → hardware)
ros2 launch bringup bringup.launch.py

# Run a specific node
ros2 run <package_name> <executable_name>
```

### Web Control App
```bash
cd ~/dogzilla/app_dogzilla

# Start (HTTP on :6500, TCP mobile-app socket on :6000)
python3 app_dogzilla.py          # normal
python3 app_dogzilla.py debug    # verbose logging

# Helper scripts
./start_app.sh    # launches in gnome-terminal (8 s delay)
./kill_dogzilla.sh
```

### Docker (ROS dev environment)
```bash
# ROS Humble container (passes /dev/ttyAMA0, /dev/video0, /dev/input)
cd ~/dogzilla/docker && ./run_humble.sh

# SLAM container
./run_slam.sh
```

### ROS 2 Linting (per package)
```bash
cd ~/dogzilla/yahboomcar_ws
colcon test --packages-select <package_name>
colcon test-result --verbose
```
Individual linters: `ament_flake8`, `ament_pep257`, `ament_copyright`

## Architecture

### Layers

```
Web/Joystick UI  ──►  app_dogzilla/ (Flask + Gevent, :6500)
Mobile App (TCP) ──►  app_dogzilla/ TCP socket (:6000)
                              │
ROS 2 Topics     ──►  yahboomcar_ws/src/ (28 ROS packages)
                              │
Hardware Lib     ──►  DOGZILLALib/DOGZILLALib.py  (serial → /dev/ttyAMA0)
```

### DOGZILLALib (hardware control)

`DOGZILLALib/DOGZILLALib/DOGZILLALib.py` — the only interface to the physical robot. Communicates over `/dev/ttyAMA0` at 115200 baud using a custom framed protocol:

- Frame format: `[0x55, 0x00, length, mode, address, ...data, checksum, 0x00, 0xAA]`
- Checksum: `255 - ((length + 0x08 + mode + address + sum(data)) % 256)`
- All motion values are encoded to 0–255 via `conver2u8()` before sending

Key parameter limits:
- Translation: X ±35 mm, Y ±18 mm, Z 75–115 mm
- Attitude: Roll ±20°, Pitch ±15°, Yaw ±11°
- Velocity: Vx ±25, Vy ±18, Vyaw ±100

Behavioural quirks to be aware of:
- `turn(step)`: values in (0, 30) are clamped to 30, (-30, 0) to -30 — there is a minimum turn speed dead zone
- `action(255)` / `reset()` — stops all motion and restores initial posture
- `pace("slow")` forces Z to 75 mm; calling `pace("normal")` or `pace("high")` does not automatically restore height

### ROS 2 Packages (`yahboomcar_ws/src/`)

**Motion & Gait:**
- `yahboom_base` — subscribes to `/cmd_vel` (geometry_msgs/Twist), translates to DOGZILLALib calls; scales by `rate=40`
- `yahboom_gait` — gait controller entry point
- `champ` / `champ_bringup` — CHAMP quadruped framework integration
- `yahboom_description` — URDF model
- `yahboom_dog_joint_state`, `yahboom_set_height` — joint state and height utilities

**Perception:**
- `yahboom_visual` — image processing pipeline
- `yahboom_color_tracking` — color-based object tracking
- `yahboom_color_identify_interfaces` / `yahboom_color_identify_server` — color identification service
- `image_color_lab` — color space utilities
- `yahboom_qrcode` / `yahboom_qrcode_tracking` — QR code detection and tracking
- `yahboom_mediapipe` — hand/pose detection via MediaPipe

**Sensors:**
- `yahboomcar_ydlidar` / `yahboom_laser` / `oradar_lidar` — lidar integrations
- `tf_publisher` — TF frame publishing

**Navigation:**
- `yahboom_bringup` — launch files for Cartographer SLAM, localization, navigation
- `bringup` — base bringup launch (including `bringup.launch.py`)
- `yahboom_app_save_map`, `yahboom_attitude_record_interfaces` — map saving, attitude recording

**Infrastructure:**
- `yahboom_msgs` — custom message types: `Target`, `TargetArray`, `Position`, `PointArray`, `ImageMsg`
- `yahboom_color_staus_redis` — Redis integration for inter-process color state
- `yahboom_publish` — sensor data publishing utilities
- `voice_xgo_ctrl_run` — voice control integration

### Key ROS Topics
- `/cmd_vel` — velocity commands (geometry_msgs/Twist); linear.x/y map to Vx/Vy, angular.z maps to Vyaw
- `/image_raw` — raw camera input
- `/image` — processed image output

### Web App (`app_dogzilla/`)

Flask + Gevent serving a camera MJPEG stream and acting as a bridge to the mobile app.

- HTTP server: port **6500** (`/` index, `/video_feed` MJPEG stream, `/init` starts TCP)
- TCP socket: port **6000** — mobile app protocol uses `$TTFFLLDDDDCC#` hex-ASCII frames
- Threads: `task_press_up_handle`, `task_joystick_handle`, `task_oled_handle` (if systemd service not active)
- Key globals: `STEP_SCALE_X/Y/Z` (motion scaling), `g_height` (body Z, default 108), `g_pace_freq` (1=slow/2=normal/3=high)
- `app_dogzilla.py debug` enables verbose print logging throughout all modules
- Auto-detects Pi IP on eth0 then wlan0 at startup

### Mobile App TCP Protocol

Frames are ASCII: `$<TYPE><CMD><LEN><DATA><CHK>#`  
All fields are 2-hex-digit bytes. Checksum = sum of all hex bytes mod 256.  
Key commands: `0F` (switch mode), `11` (joystick move), `12` (button), `21` (attitude), `22` (height), `31` (action), `41` (motor), `51` (leg).

## Example Code

`Samples/` contains standalone Jupyter notebooks organized by category:
- `2_Control/` — direct robot control via DOGZILLALib (best starting point for hardware experiments)
- `3_AI_Visual/` — color tracking, face detection, QR code, obstacle crossing
- `4_Big_Modle/` — LLM integration (see `API_KEY.py` for key config)

## Hardware Notes

- Serial port: `/dev/ttyAMA0` (Raspberry Pi GPIO UART)
- Target OS: Raspberry Pi 5 running Ubuntu with ROS 2 Humble
- Camera: `/dev/video0`, auto-fallback to `/dev/video1`
- USB joystick: `/dev/input/js0` (Xbox-style layout)
- OLED display managed by `oled_dogzilla.py`; also runs as `yahboom_oled.service` systemd unit
- The STEP 3D model is at `dogzilla_s2.step`
- `fastdds_unicast.xml` configures DDS for the Docker ROS environment

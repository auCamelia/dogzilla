# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DOGZILLA is a 12-DOF quadruped robot (by Yahboom Technology) powered by Raspberry Pi 5. It combines hardware servo control via serial, a Flask-based web control app, and a ROS 2 (Humble) workspace for autonomous navigation and perception.

## Build & Run Commands

### ROS 2 Workspace
```bash
# Build entire workspace
cd ~/dogzilla/dogzilla_ws
colcon build

# Build a single package
colcon build --packages-select <package_name>

# Source the workspace after building
source install/setup.bash

# Launch the base robot
ros2 launch yahboom_bringup bringup.launch.py

# Run a specific node
ros2 run <package_name> <executable_name>
```

### Web Control App
```bash
# Start the Flask app
cd ~/dogzilla/app_dogzilla
python3 app_dogzilla.py

# Or via the helper script
./start_app.sh

# Stop the app
./kill_dogzilla.sh
```

### ROS 2 Linting (per package)
```bash
cd ~/dogzilla/dogzilla_ws
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Individual linters: `ament_flake8`, `ament_pep257`, `ament_copyright`

## Architecture

### Layers

```
Web/Joystick UI  ──►  app_dogzilla/ (Flask + Gevent)
                              │
ROS 2 Topics     ──►  dogzilla_ws/src/ (29 ROS packages)
                              │
Hardware Lib     ──►  DOGZILLALib/DOGZILLALib.py  (serial → /dev/ttyAMA0)
```

### DOGZILLALib (hardware control)

`DOGZILLALib/DOGZILLALib/DOGZILLALib.py` — the only interface to the physical robot. Communicates over `/dev/ttyAMA0` at 115200 baud using a custom framed protocol:

- Frame format: `[0x55, 0x00, length, mode, address, ...data, checksum, 0x00, 0xAA]`
- Checksum: `255 - ((length + 0x08 + mode + address + sum(data)) % 256)`

Key parameter limits:
- Translation: X ±35 mm, Y ±18 mm, Z 75–115 mm
- Attitude: Roll ±20°, Pitch ±15°, Yaw ±11°
- Velocity: Vx ±25, Vy ±18, Vyaw ±100

### ROS 2 Packages (`dogzilla_ws/src/`)

**Motion & Gait:**
- `yahboom_base` — subscribes to `/cmd_vel` (geometry_msgs/Twist), translates to DOGZILLALib calls
- `yahboom_gait` — gait controller entry point
- `champ` / `champ_bringup` — CHAMP quadruped framework integration
- `yahboom_description` — URDF model

**Perception:**
- `yahboom_visual` — 7-node image processing pipeline (pub_image, laser_to_image, AR overlay, depth handling)
- `yahboom_color_tracking` — color-based object tracking
- `yahboom_qrcode_tracking` — QR code detection and tracking
- `yahboom_mediapipe` — hand/pose detection via MediaPipe

**Sensors:**
- `yahboomcar_ydlidar` / `yahboom_laser` / `oradar_lidar` — lidar integrations
- `tf_publisher` — TF frame publishing

**Infrastructure:**
- `yahboom_msgs` — custom message types: `Target`, `TargetArray`, `Position`, `PointArray`, `ImageMsg`
- `bringup` / `yahboom_bringup` — launch files and startup configuration
- `yahboom_color_staus_redis` — Redis integration for inter-process color state

### Key ROS Topics
- `/cmd_vel` — velocity commands (geometry_msgs/Twist)
- `/image_raw` — raw camera input
- `/image` — processed image output

### Web App (`app_dogzilla/`)

Flask + Gevent serving a real-time camera stream and joystick control. Key globals: `STEP_SCALE_X/Y/Z`, `g_height`, `pace_freq`. Auto-detects the Pi's IP address at startup.

## Example Code

`Samples/` contains standalone scripts organized by category:
- `1_OpenCV/` — basic vision
- `2_Control/` — direct robot control via DOGZILLALib
- `3_AI_Visual/` — AI-augmented vision
- `4_Big_Modle/` — LLM integration examples

## Hardware Notes

- Serial port: `/dev/ttyAMA0` (Raspberry Pi GPIO UART)
- Target OS: Raspberry Pi 5 running Ubuntu with ROS 2 Humble
- The STEP 3D model is at `DOGZILLA S2.STEP`

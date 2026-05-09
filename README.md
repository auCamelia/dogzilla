<div align="center">

# 🐕 DOGZILLA

**Autonomous 12-DOF Quadruped Robot — ROS 2 · Raspberry Pi 5 · Docker**

<p>
  <img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%205-C51A4A?style=for-the-badge&logo=raspberry-pi&logoColor=white"/>
  <img src="https://img.shields.io/badge/Docker-ARM64-2496ED?style=for-the-badge&logo=docker&logoColor=white"/>
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
</p>

*Browser-based teleop · SLAM mapping · Autonomous navigation · Digital twin in RViz2*

</div>

---

## Architecture

```mermaid
graph TB
    subgraph PC["💻  PC — ROS 2 Jazzy (native)"]
        direction TB
        Browser["🌐 Browser\nteleop.html :8080"]
        RB["rosbridge_server :9090"]
        Nav2["🧭 Nav2\n(autonomous navigation)"]
        SLAM_PC["🗺️ slam_toolbox\n(map consumption)"]
        RViz["📊 RViz2 — Digital Twin"]
        Browser -- roslibjs WebSocket --> RB
    end

    subgraph BRIDGE["🔀  ROS 2 bridge layer  ·  always in the middle"]
        direction LR
        B1["yahboom_ctrl\n(real robot)"]
        B2["gz_ros2_bridge\n(Gazebo simulation)"]
    end

    subgraph NET["🔗  LAN · ROS_DOMAIN_ID=0 · FastDDS unicast"]
        direction LR
        T1["/cmd_vel\n/dogzilla/*"]
        T2["/scan · /odom · /tf · /map"]
    end

    subgraph PI["🍓  Raspberry Pi 5 — Docker dogzilla:jazzy"]
        direction TB
        SLAM_PI["🗺️ slam_toolbox\n(cartography)"]
        LIB["⚙️ DOGZILLALib\nserial /dev/ttyAMA0"]
        HW["🤖 Hardware — 12 DOF"]
        LIDAR["📡 LiDAR"]
        B1 --> LIB --> HW
        LIDAR --> SLAM_PI
    end

    subgraph SIM["🖥️  Gazebo (optional)"]
        GZ["Simulated robot\n+ sensors"]
        B2 --> GZ
    end

    RB  -- topics --> T1
    Nav2 -- /cmd_vel --> T1
    T1  --> B1
    T1  -.->|swap bridge| B2
    T2  --> RViz
    T2  --> SLAM_PC
    SLAM_PI -- /map /tf --> T2
    LIDAR   -- /scan    --> T2
```

> **Key design principle** — the bridge layer is always between the PC and the hardware.  
> Teleop, Nav2 and any autonomous behaviour publish **only ROS 2 topics**.  
> Swapping `yahboom_ctrl` for `gz_ros2_bridge` gives a full Gazebo simulation with zero PC-side changes.

---

## Teleop Interface

<div align="center">

![Teleop UI](https://img.shields.io/badge/Interface-Web%20Browser-00aa50?style=flat-square&logo=googlechrome&logoColor=white)

</div>

A dark-themed single-page app served at `http://localhost:8080/teleop.html`:

| Zone | Controls |
|---|---|
| **D-pad** | `Z`/↑ fwd · `S`/↓ back · `Q`/← left · `D`/→ right · `A` turn-L · `E` turn-R · `Space` stop |
| **Pace** | `F1` slow · `F2` normal · `F3` high (or click buttons) |
| **Actions** | `1`–`9` keys or click — 19 motions (Stand Up, Crawl, Wave, Handshake …) |
| **Reset** | `0` — restores initial posture |
| **Sliders** | Translation X/Y/Z (mm) · Attitude Roll/Pitch/Yaw (°) |

Connection URL is editable in the header — switch from `ws://localhost:9090` to `ws://<pi-ip>:9090` to drive the real robot.

---

## Quick Start

### 1 — PC setup

```bash
# ROS 2 Jazzy + rosbridge
sudo apt install ros-jazzy-desktop ros-jazzy-rosbridge-server

# Build the teleop package
cd ~/dogzilla/yahboomcar_ws
colcon build --packages-select dogzilla_teleop
source install/setup.bash
```

### 2 — Build & transfer the Docker image to the Pi

> Build happens on the PC (x86 → ARM64 cross-compilation via QEMU + buildx).  
> The Pi image contains only: hardware bridge, sensors, slam-toolbox. **Nav2 is on the PC.**

```bash
# One-time setup
sudo apt install docker-buildx
docker buildx create --name multiarch --use
docker buildx inspect --bootstrap

# Build ARM64 image (~30 min first time)
cd ~/dogzilla
docker buildx build \
  --platform linux/arm64 \
  -f docker/Dockerfile.jazzy \
  -t dogzilla:jazzy \
  --output type=docker,dest=/tmp/dogzilla_jazzy_arm64.tar \
  .

# Transfer to Pi
scp /tmp/dogzilla_jazzy_arm64.tar pi@<pi-ip>:~
ssh pi@<pi-ip> docker load -i dogzilla_jazzy.tar
```

### 3 — Launch teleop

**PC**
```bash
source /opt/ros/jazzy/setup.bash
source ~/dogzilla/yahboomcar_ws/install/setup.bash

ros2 launch dogzilla_teleop teleop.launch.py
# → opens http://localhost:8080/teleop.html automatically
```

**Pi** (inside Docker container)
```bash
./docker/run_jazzy.sh

# inside the container:
ros2 launch yahboom_base yahboom_base.launch.py
```

Change the rosbridge URL in the browser to `ws://<pi-ip>:9090` and the dot turns green.

---

## SLAM Mapping

```bash
# Pi — start SLAM
./docker/run_jazzy.sh slam

# PC — visualise in RViz2
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
rviz2   # add: Map · LaserScan · RobotModel · TF
```

Drive the robot with the teleop interface while the map builds in RViz2.

**Record a bag for offline SLAM tuning:**
```bash
ros2 bag record /scan /odom /tf /tf_static -o slam_session
ros2 bag play slam_session   # replay as many times as needed
```

---

## Autonomous Navigation

Nav2 runs on the **PC** (more compute, keeps the Pi lean).  
`yahboom_ctrl` on the Pi simply consumes `/cmd_vel` — same as teleop.

```bash
# PC — install Nav2 (once)
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Pi — robot mode only (no Nav2 in Docker image)
./docker/run_jazzy.sh

# PC — launch Nav2 with a saved map
ros2 launch dogzilla_nav navigation.launch.py map:=/path/to/map.yaml
```

Set a **2D Nav Goal** in RViz2 — Nav2 plans the path and sends `/cmd_vel` to the Pi.  
Set a 2D Nav Goal in RViz2 and the robot walks there on its own.

---

## PC ↔ Pi Networking

Both machines must share `ROS_DOMAIN_ID=0` on the same LAN.

```bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
```

`fastdds_unicast.xml` disables multicast (required on most Wi-Fi networks).  
Edit `<address>` inside to set the Pi's static IP if needed.

---

## ROS 2 Topics & Services

### Motion Control

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Teleop UI · Nav2 · laser nodes · color/QR trackers | `yahboom_ctrl` | main locomotion command |
| `/dogzilla/action` | `std_msgs/Int32` | Teleop UI (rosbridge) | `yahboom_ctrl` | 1–19 motions · 255=reset |
| `/dogzilla/pace` | `std_msgs/String` | Teleop UI (rosbridge) | `yahboom_ctrl` | `slow` / `normal` / `high` |
| `/dogzilla/translation` | `geometry_msgs/Vector3` | Teleop UI (rosbridge) | `yahboom_ctrl` | x±35mm y±18mm z75-115mm |
| `/dogzilla/attitude` | `geometry_msgs/Vector3` | Teleop UI (rosbridge) | `yahboom_ctrl` | roll±20° pitch±15° yaw±11° |
| `/JoyState` | `std_msgs/Bool` | joystick node | laser tracker / avoidance / warning | enables autonomous laser modes |
| `/Buzzer` | `std_msgs/Bool` | laser warning nodes | hardware | proximity alarm |

### Sensors & State

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | oradar/ydlidar driver | slam_toolbox · laser nodes | main mapping input |
| `/joint_states` | `sensor_msgs/JointState` | `yahboomcar_joint_state` | robot_state_publisher | 12 DOF · lf/lh/rf/rh hip · upper/lower leg |
| `/imu/data_raw_self` | `sensor_msgs/Imu` | `yahboomcar_joint_state` | Nav2 (PC) | orientation as quaternion from roll/pitch/yaw |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox (Pi) | Nav2 · RViz2 (PC) | built during SLAM phase |
| `/tf` · `/tf_static` | — | `tf_publisher` · robot_state_publisher | RViz2 · Nav2 | `base_footprint → base_link` (x:0.115m z:0.047m) |

### Perception

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | `yahboom_publish` (C++ node) | qrcode tracker | raw camera MJPEG |
| `/image_contours` | `sensor_msgs/CompressedImage` | `yahboom_publish` | `yahboom_color_tracking` | contours drawn over frame |
| `/obj_msg` | `image_color_lab/StringStamped` | `yahboom_publish` | color tracker · QR tracker | JSON: area, center_x/y, img_w/h, movement |
| `/lab_set` | `std_msgs/String` | external / UI | `yahboom_publish` | LAB color range params for segmentation |
| `/mediapipe/points` | `yahboom_msgs/PointArray` | HandDetector · PoseDetector · Holistic · FaceMesh | any consumer | MediaPipe landmarks (hand/pose/face) |

### Services

| Service | Type | Server | Client | Notes |
|---|---|---|---|---|
| `yahboomSetAttiude` | `yahboom_attitude_record_interfaces/AttuitudeRecord` | attitude record node | `yahboom_color_tracking` | set pitch/yaw during tracking · req: pitch+yaw int64 · res: string |
| `yahboomColorIdentify` | `yahboom_color_identify_interfaces/ColorIdentify` | `yahboom_color_identify_server` | color tracking nodes | identify dominant color in frame · req: if_identify string |

---

## Repository Layout

```
dogzilla/
├── DOGZILLALib/              hardware library — serial framing to /dev/ttyAMA0
├── app_dogzilla/             legacy Flask app (port 6500)
├── docker/
│   ├── Dockerfile.jazzy      ROS Jazzy + slam-toolbox + nav2 (ARM64)
│   └── run_jazzy.sh          container launcher — modes: robot / slam / nav
├── samples/                  Jupyter notebooks (control, vision, LLM)
├── yahboomcar_ws/src/
│   ├── dogzilla_teleop/      web teleop interface (PC only)
│   ├── yahboom_base/         hardware bridge — yahboom_ctrl node (Pi)
│   ├── yahboom_bringup/      SLAM + Nav2 launch files
│   ├── yahboom_description/  URDF model
│   └── …                     20+ additional ROS 2 packages
├── fastdds_unicast.xml       DDS peer discovery for local network
└── CLAUDE.md                 AI coding assistant guide
```

---

<div align="center">
<sub>Built with ROS 2 Jazzy · Yahboom Dogzilla S2 · Raspberry Pi 5</sub>
</div>

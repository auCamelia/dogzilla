<div align="center">

# 🐕 DOGZILLA

**Autonomous 12-DOF Quadruped Robot — ROS 2 · Raspberry Pi 5 · Docker**

<p>
  <img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%205-C51A4A?style=for-the-badge&logo=raspberry-pi&logoColor=white"/>
  <img src="https://img.shields.io/badge/Docker-ARM64-2496ED?style=for-the-badge&logo=docker&logoColor=white"/>
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
</p>

</div>

---

## Deux modes de fonctionnement

| | Mode Classic | Mode ROS 2 |
|---|---|---|
| **Pi** | `app_dogzilla.py` direct | Docker `dogzilla:jazzy` |
| **Contrôle** | App Yahboom (smartphone) | Browser (PC · téléphone · montre) |
| **ROS 2** | Non | Oui |
| **Jumeau numérique** | Non | RViz2 sur PC |
| **Navigation autonome** | Non | Nav2 sur le Pi |

---

## Mode Classic — sans ROS 2

Le Pi contrôle le hardware directement via la lib série. Aucun Docker, aucun ROS 2.

**Pi**
```bash
cd ~/app_dogzilla
python3 app_dogzilla.py
```

Ouvrir l'app Yahboom sur le smartphone → connecter sur `<pi-ip>:6000`.

> Ne pas lancer Docker en parallèle — `app_dogzilla.py` et `yahboom_ctrl` utilisent tous les deux `/dev/ttyAMA0`.

---

## Mode ROS 2 — Docker

### Architecture

```
Browser / RViz2 / Programme mission
        │
        │  WebSocket :9090 (rosbridge)
        │  DDS (ROS 2 topics)
        ▼
┌──────────────────────────────────────────┐
│  Pi — container Docker dogzilla:jazzy    │
│                                          │
│  yahboom_ctrl  ←── /cmd_vel             │
│  robot_state_publisher                   │
│  yahboomcar_joint_state                  │
│  usb_cam       ──► /image_raw/compressed │
│  rosbridge_websocket  :9090              │
│  dogzilla_web_server  :8080              │
│                                          │
│  [mode slam]  slam_toolbox + oradar      │
│  [mode nav]   oradar + rf2o + Nav2       │
└──────────────────────────────────────────┘
        │
        │  /dev/ttyAMA0 (serial 115200)
        ▼
   Hardware Dogzilla S2
```

### Setup initial sur le Pi (une seule fois)

```bash
# Cloner le repo
git clone <repo> ~/dogzilla

# Construire le workspace ROS 2 dans le container (natif ARM64, ~15 min)
cd ~/dogzilla
./docker/run_jazzy.sh --build
```

Le build produit `install/` `build/` `log/` dans `yahboomcar_ws/` — persistés sur le Pi via volume mount. À refaire après un `git pull` qui touche des fichiers Python ou des launch files.

> Les fichiers HTML/JS (`teleop.html`, `watch.html`) sont live via le volume mount — un `git pull` suffit, pas besoin de rebuilder.

### Commandes Pi

```bash
./docker/run_jazzy.sh [--robot]              # robot (défaut) — téléopération
./docker/run_jazzy.sh --slam                 # cartographie SLAM
./docker/run_jazzy.sh --nav [map.yaml]       # navigation autonome Nav2
./docker/run_jazzy.sh --build                # colcon build workspace
```

---

## Téléopération

Depuis n'importe quel device sur le même réseau :

```
http://<pi-ip>:8080/teleop.html   ← PC · téléphone · tablette
http://<pi-ip>:8080/watch.html    ← Samsung Watch
```

Le browser se connecte automatiquement à `ws://<pi-ip>:9090`. Le point dans l'en-tête vire au vert quand la connexion est établie. La caméra est streamée via rosbridge (`/image_raw/compressed`).

---

## Jumeau numérique sur le PC

Le PC se connecte au même domaine ROS 2 via DDS unicast.

**PC — une seule fois**
```bash
sudo apt install ros-jazzy-desktop
cd ~/dogzilla/yahboomcar_ws
colcon build --packages-select yahboom_description
```

**PC — à chaque session**
```bash
source /opt/ros/jazzy/setup.bash
source ~/dogzilla/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml

rviz2
```

Dans RViz2 : **RobotModel** · **TF** · **LaserScan** · **Map** · **Path** · costmaps.

---

## SLAM — construire une carte

```bash
# Pi
./docker/run_jazzy.sh --slam

# PC — visualiser la carte en construction
source /opt/ros/jazzy/setup.bash && source ~/dogzilla/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
rviz2   # ajouter : Map · LaserScan · RobotModel · TF
```

Piloter le robot depuis le browser pour couvrir la zone. Sauvegarder la carte :

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/ma_carte
# → /home/pi/maps/ma_carte.yaml + ma_carte.pgm
```

---

## Navigation autonome (Nav2)

Nav2 tourne entièrement sur le Pi. Le PC n'est nécessaire que pour visualiser (RViz2) ou envoyer des missions (programme Python/ROS 2).

```
Programme mission (PC)
    │  NavigateToPose action
    ▼
Nav2 (Pi) ── /map · /scan · /odom (rf2o) ◄── oradar LiDAR
    │
    │  /cmd_vel
    ▼
yahboom_ctrl (Pi) → hardware
```

```bash
# Pi — lancer la navigation avec une carte existante
./docker/run_jazzy.sh --nav /root/maps/ma_carte.yaml

# ou avec la carte par défaut (/root/maps/map.yaml)
./docker/run_jazzy.sh --nav
```

**Depuis RViz2** : bouton **2D Nav Goal** → cliquer sur la carte → le robot y va seul.

**Depuis un programme Python** :
```python
# Envoyer un goal NavigateToPose via rclpy ou via rosbridge WebSocket
```

Les paramètres Nav2 sont dans `yahboomcar_ws/src/yahboom_bringup/config/nav2_params.yaml`.  
Valeurs clés : `max_velocity: [0.25, 0.0, 1.0]` (m/s), `robot_radius: 0.15` m, `inflation_radius: 0.35` m.

---

## Réseau PC ↔ Pi

Les deux machines doivent être sur le même LAN avec `ROS_DOMAIN_ID=0`.

```bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
```

`fastdds_unicast.xml` désactive le multicast (obligatoire sur la plupart des réseaux Wi-Fi).  
Éditer `<address>` dans ce fichier si l'IP du Pi change.

---

## Construire et transférer l'image Docker

> À faire sur le PC à chaque modification du Dockerfile (ajout de paquet apt, etc.).  
> Le workspace ROS 2 **n'est pas** baked dans l'image — il est reconstruit sur le Pi via `--build`.

```bash
# Setup une fois
sudo apt install docker-buildx
docker buildx create --name multiarch --use
docker buildx inspect --bootstrap

# Build (~30 min première fois)
cd ~/dogzilla
docker buildx build \
  --platform linux/arm64 \
  -f docker/Dockerfile.jazzy \
  -t dogzilla:jazzy \
  --output type=docker,dest=/tmp/dogzilla_jazzy_arm64.tar \
  .

# Transférer sur le Pi
scp /tmp/dogzilla_jazzy_arm64.tar pi@<pi-ip>:~
ssh pi@<pi-ip> "docker load -i dogzilla_jazzy_arm64.tar"

# Puis rebuild du workspace sur le Pi
ssh pi@<pi-ip> "cd ~/dogzilla && ./docker/run_jazzy.sh --build"
```

---

## Référence des nœuds ROS 2

### Pi — container Docker (tous modes)

| Nœud | Package | Rôle |
|---|---|---|
| `yahboom_ctrl` | `yahboom_base` | Traduit `/cmd_vel` + `/dogzilla/*` → DOGZILLALib → série `/dev/ttyAMA0` |
| `robot_state_publisher` | `robot_state_publisher` | URDF → `/tf_static` |
| `yahboomcar_joint_state` | `yahboom_dog_joint_state` | Angles servos + IMU → `/joint_states` + `/imu/data_raw_self` |
| `usb_cam_node_exe` | `usb_cam` | `/dev/video0` → `/image_raw` + `/image_raw/compressed` |
| `rosbridge_websocket` | `rosbridge_server` | WebSocket `:9090` — JSON roslibjs ↔ topics DDS |
| `dogzilla_web_server` | `dogzilla_teleop` | HTTP `:8080` — sert `teleop.html`, `watch.html` |

### Pi — mode SLAM (`--slam`)

| Nœud | Package | Rôle |
|---|---|---|
| `oradar_scan` | `oradar_lidar` | LiDAR MS200 `/dev/ttyAMA1` → `/scan` + TF `base_link→laser_frame` |
| `slam_toolbox` | `slam_toolbox` | Fusionne `/scan` + `/tf` → construit et publie `/map` |

### Pi — mode navigation (`--nav`)

| Nœud | Package | Rôle |
|---|---|---|
| `oradar_scan` | `oradar_lidar` | LiDAR MS200 → `/scan` + TF `base_link→laser_frame` |
| `rf2o_laser_odometry` | `rf2o_laser_odometry` | Scan matching consécutif → `/odom` + TF `odom→base_footprint` |
| `nav2_bringup` | `nav2_bringup` | AMCL + map server + planner + controller + costmaps |

### PC (optionnel)

| Composant | Rôle |
|---|---|
| `rviz2` | Jumeau numérique — RobotModel, TF, LaserScan, Map, costmap, trajectoires, 2D Nav Goal |
| Programme mission | Publie sur `NavigateToPose` action ou `/move_base_simple/goal` |

### Nœuds de perception (optionnels, à lancer manuellement)

| Nœud | Package | Rôle |
|---|---|---|
| `yahboom_color_tracking` | `yahboom_color_tracking` | Suit un objet coloré → publie `/cmd_vel` |
| `yahboom_qrcode_tracking` | `yahboom_qrcode_tracking` | Détecte et suit un QR code |
| `yahboom_mediapipe` | `yahboom_mediapipe` | Landmarks main/pose/visage → `/mediapipe/points` |
| `yahboom_publish` (C++) | `yahboom_publish` | Pipeline caméra → `/image_raw/compressed`, `/image_contours`, `/obj_msg` |
| Laser tracker / avoider | `yahboom_laser` | Évitement d'obstacles et suivi d'objet par LiDAR |

---

## Topics principaux

| Topic | Type | De | Vers |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Teleop · Nav2 · trackers | `yahboom_ctrl` |
| `/dogzilla/action` | `std_msgs/Int32` | Teleop (rosbridge) | `yahboom_ctrl` — motions 1-19, 255=reset |
| `/dogzilla/pace` | `std_msgs/String` | Teleop (rosbridge) | `yahboom_ctrl` — `slow`/`normal`/`high` |
| `/dogzilla/translation` | `geometry_msgs/Vector3` | Teleop (rosbridge) | `yahboom_ctrl` — x±35mm y±18mm z 75-115mm |
| `/dogzilla/attitude` | `geometry_msgs/Vector3` | Teleop (rosbridge) | `yahboom_ctrl` — roll±20° pitch±15° yaw±11° |
| `/battery_voltage` | `std_msgs/Float32` | `yahboom_ctrl` | Teleop (affiché dans le header) |
| `/scan` | `sensor_msgs/LaserScan` | `oradar_scan` | `slam_toolbox` · Nav2 costmaps · AMCL |
| `/odom` | `nav_msgs/Odometry` | `rf2o_laser_odometry` | Nav2 (AMCL, costmaps) |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` · Nav2 map server | Nav2 · RViz2 |
| `/joint_states` | `sensor_msgs/JointState` | `yahboomcar_joint_state` | `robot_state_publisher` |
| `/imu/data_raw_self` | `sensor_msgs/Imu` | `yahboomcar_joint_state` | Nav2 |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | `usb_cam` | rosbridge → teleop.html |
| `/tf` · `/tf_static` | — | `robot_state_publisher` · `oradar_scan` | RViz2 · Nav2 |

---

## Structure du dépôt

```
dogzilla/
├── DOGZILLALib/              bibliothèque hardware — framing série vers /dev/ttyAMA0
├── app_dogzilla/             app Flask mode Classic (ports 6500 HTTP · 6000 TCP)
├── docker/
│   ├── Dockerfile.jazzy      image Pi : ROS Jazzy + Nav2 + rf2o + rosbridge
│   ├── entrypoint.sh         mode robot : yahboom_ctrl + capteurs + rosbridge + web
│   ├── entrypoint_slam.sh    mode SLAM  : + slam_toolbox
│   ├── entrypoint_nav.sh     mode nav   : + oradar + rf2o + Nav2
│   ├── entrypoint_build.sh   mode build : colcon build workspace sur Pi
│   └── run_jazzy.sh          lanceur — modes : --robot · --slam · --nav · --build
├── samples/                  notebooks Jupyter (contrôle, vision, LLM)
├── yahboomcar_ws/src/
│   ├── dogzilla_teleop/      interface web — teleop.html + watch.html
│   ├── yahboom_base/         bridge hardware — nœud yahboom_ctrl
│   ├── yahboom_bringup/
│   │   └── config/
│   │       └── nav2_params.yaml   paramètres Nav2 calibrés pour le Dogzilla S2
│   ├── yahboom_description/  modèle URDF
│   └── …                     20+ packages ROS 2 (perception, laser, mediapipe…)
├── fastdds_unicast.xml       discovery DDS en unicast (Wi-Fi sans multicast)
└── CLAUDE.md                 guide pour l'assistant IA
```

---

<div align="center">
<sub>ROS 2 Jazzy · Yahboom Dogzilla S2 · Raspberry Pi 5</sub>
</div>

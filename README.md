# Dogzilla — ROS 2 Architecture

12-DOF quadruped robot (Yahboom) piloté par Raspberry Pi 5.  
Stack ROS 2 Jazzy : jumeau numérique sur PC, bridge hardware sur Pi.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  PC  (ROS 2 Jazzy natif)                                │
│                                                         │
│  Browser ──roslibjs──► rosbridge_server (:9090)         │
│  http://localhost:8080/teleop.html                      │
│                │                                        │
│                ▼                                        │
│         topics ROS 2                                    │
│   /cmd_vel · /dogzilla/action                          │
│   /dogzilla/pace · /dogzilla/translation               │
│   /dogzilla/attitude                                    │
│                │                                        │
│         RViz2 (jumeau numérique)                        │
│   /scan · /odom · /tf · /map                           │
└──────────────────┬──────────────────────────────────────┘
                   │  réseau local (ROS_DOMAIN_ID=0)
                   │  DDS unicast → fastdds_unicast.xml
┌──────────────────┴──────────────────────────────────────┐
│  Pi 5  (Docker dogzilla:jazzy)                          │
│                                                         │
│  yahboom_ctrl ◄── /cmd_vel, /dogzilla/*                │
│       │                                                 │
│       ▼                                                 │
│  DOGZILLALib ──serial /dev/ttyAMA0──► hardware          │
│                                                         │
│  Mode SLAM : slam_toolbox → /map + /tf                 │
│  Mode Nav  : nav2         → /cmd_vel (autonome)        │
└─────────────────────────────────────────────────────────┘
```

---

## Packages ROS 2

| Package | Rôle | Côté |
|---|---|---|
| `dogzilla_teleop` | Serveur web + rosbridge → interface de téléop | PC |
| `yahboom_base` / `yahboom_ctrl` | Bridge ROS 2 ↔ DOGZILLALib (hardware) | Pi |
| `yahboom_bringup` | SLAM (Cartographer), localisation, Nav2 | Pi |
| `yahboom_description` | URDF du robot | PC + Pi |
| `bringup` | Bringup de base (cmd_vel → hardware) | Pi |

### Topics principaux

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | PC → Pi |
| `/dogzilla/action` | `std_msgs/Int32` | PC → Pi (1–19, 255=reset) |
| `/dogzilla/pace` | `std_msgs/String` | PC → Pi (`slow`/`normal`/`high`) |
| `/dogzilla/translation` | `geometry_msgs/Vector3` | PC → Pi (x±35 y±18 z75-115 mm) |
| `/dogzilla/attitude` | `geometry_msgs/Vector3` | PC → Pi (roll±20° pitch±15° yaw±11°) |
| `/scan` | `sensor_msgs/LaserScan` | Pi → PC |
| `/odom` | `nav_msgs/Odometry` | Pi → PC |
| `/tf`, `/tf_static` | — | Pi → PC |
| `/map` | `nav_msgs/OccupancyGrid` | Pi → PC (mode SLAM/Nav) |

---

## Lancement

### Prérequis PC

```bash
# ROS 2 Jazzy natif + rosbridge
sudo apt install ros-jazzy-desktop ros-jazzy-rosbridge-server

# Builder le workspace (dogzilla_teleop uniquement sur PC)
cd ~/dogzilla/yahboomcar_ws
colcon build --packages-select dogzilla_teleop
source install/setup.bash
```

### Prérequis Pi — builder et transférer l'image Docker

```bash
# Sur le PC (nécessite docker buildx + QEMU aarch64)
sudo apt install docker-buildx-plugin
docker buildx create --name multiarch --use
docker buildx inspect --bootstrap

cd ~/dogzilla
docker buildx build --platform linux/arm64 \
  -f docker/Dockerfile.jazzy \
  -t dogzilla:jazzy \
  --output type=docker,dest=dogzilla_jazzy.tar .

scp dogzilla_jazzy.tar pi@<ip-pi>:~
ssh pi@<ip-pi> docker load -i dogzilla_jazzy.tar
```

---

### 1 — Téléopération (PC + Pi)

**Sur le PC :**
```bash
source /opt/ros/jazzy/setup.bash
source ~/dogzilla/yahboomcar_ws/install/setup.bash

# Lance rosbridge (:9090) + serveur web (:8080) + ouvre le browser
ros2 launch dogzilla_teleop teleop.launch.py
```

Ouvrir `http://localhost:8080/teleop.html` — le point passe au vert dès que rosbridge est joignable.

**Sur le Pi (dans le container Docker) :**
```bash
./docker/run_jazzy.sh

# Dans le container :
ros2 launch yahboom_base yahboom_base.launch.py
```

Pour piloter depuis le browser, changer l'URL rosbridge en `ws://<ip-pi>:9090`.

### Contrôles clavier (interface web)

| Touche | Action |
|---|---|
| `Z` / `↑` | Avancer |
| `S` / `↓` | Reculer |
| `Q` / `←` | Latéral gauche |
| `D` / `→` | Latéral droit |
| `A` | Pivoter gauche |
| `E` | Pivoter droit |
| `Espace` | Stop |
| `1`–`9` | Actions (Lie Down → Handshake) |
| `0` | Reset posture |
| `F1` / `F2` / `F3` | Pace slow / normal / high |

---

### 2 — Cartographie SLAM

**Sur le Pi :**
```bash
./docker/run_jazzy.sh slam
```

**Sur le PC — RViz2 :**
```bash
source /opt/ros/jazzy/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
rviz2
```

Ajouter les displays : `Map`, `LaserScan`, `RobotModel`, `TF`.

**Enregistrer un bag pour rejouer le SLAM hors-ligne :**
```bash
ros2 bag record /scan /odom /tf /tf_static -o slam_session
ros2 bag play slam_session
```

### 3 — Navigation autonome (Nav2)

```bash
# Sur le Pi (avec une carte existante)
./docker/run_jazzy.sh nav /root/maps/map.yaml
```

Nav2 publie sur `/cmd_vel` — le bridge `yahboom_ctrl` le consomme automatiquement.

---

## Communication PC ↔ Pi

Les deux machines doivent être sur le même réseau avec `ROS_DOMAIN_ID=0`.

```bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
```

Le fichier `fastdds_unicast.xml` est déjà configuré pour éviter le multicast (requis sur certains réseaux Wi-Fi). Remplacer `<address>` par l'IP du Pi si nécessaire.

---

## Structure du repo

```
dogzilla/
├── DOGZILLALib/          — bibliothèque hardware (serial → /dev/ttyAMA0)
├── app_dogzilla/         — app Flask legacy (port 6500)
├── docker/
│   ├── Dockerfile.jazzy  — image ROS Jazzy + slam-toolbox + nav2
│   └── run_jazzy.sh      — lance le container (modes: robot / slam / nav)
├── samples/              — notebooks Jupyter de référence
├── yahboomcar_ws/src/
│   ├── dogzilla_teleop/  — interface web de téléop (PC)
│   ├── yahboom_base/     — bridge /cmd_vel + yahboom_ctrl (Pi)
│   ├── yahboom_bringup/  — SLAM + Nav2 launch files
│   └── ...               — 25 autres packages ROS 2
├── fastdds_unicast.xml   — config DDS pour réseau local
└── CLAUDE.md             — guide pour Claude Code
```

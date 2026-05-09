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

## Two operating modes

| | Mode Classic | Mode ROS 2 |
|---|---|---|
| **Pi** | `app_dogzilla.py` direct | Docker `dogzilla:jazzy` |
| **Contrôle** | App Yahboom (smartphone) | Browser (PC · téléphone · montre) |
| **ROS 2** | Non | Oui |
| **Jumeau numérique** | Non | RViz2 sur PC |
| **Navigation autonome** | Non | Nav2 sur PC |

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

## Mode ROS 2 — Docker + jumeau numérique

### Ce qui tourne sur le Pi (Docker)

```bash
# Sur le Pi — lance tout automatiquement
./docker/run_jazzy.sh
```

Le container démarre dans l'ordre :

| Composant | Rôle |
|---|---|
| `yahboom_ctrl` | Bridge hardware — traduit `/cmd_vel` + `/dogzilla/*` en frames série vers `/dev/ttyAMA0` |
| `robot_state_publisher` | Publie les TF statiques à partir du URDF |
| `yahboomcar_joint_state` | Lit les angles servos + IMU, publie `/joint_states` et `/imu/data_raw_self` |
| `usb_cam` | Ouvre `/dev/video0`, publie `/image_raw` |
| `rosbridge_websocket` | Pont WebSocket `:9090` entre le browser (roslibjs) et les topics ROS 2 |
| `dogzilla_web_server` | Serveur HTTP `:8080` — sert `teleop.html` et `watch.html` |

### Téléopération depuis n'importe quel client

```
http://<pi-ip>:8080/teleop.html   ← PC · téléphone · tablette
http://<pi-ip>:8080/watch.html    ← Samsung Watch
```

Le browser se connecte automatiquement à `ws://<pi-ip>:9090`. Le point de couleur dans l'en-tête vire au vert quand la connexion est établie.

### Jumeau numérique sur le PC

Le PC se connecte au même domaine ROS 2 via le réseau local. Il reçoit les topics du Pi en temps réel.

**PC — une seule fois**
```bash
sudo apt install ros-jazzy-desktop
```

**PC — à chaque session**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml

rviz2
```

Dans RViz2, ajouter : **RobotModel** · **TF** · **LaserScan** · **Map** (après un SLAM).  
Le robot virtuel suit les mouvements du robot réel en temps réel.

---

## SLAM — construire une carte

```bash
# Pi — mode cartographie
./docker/run_jazzy.sh slam

# PC — visualiser la carte qui se construit
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
rviz2   # ajouter : Map · LaserScan · RobotModel · TF
```

Piloter le robot avec le browser pendant que la carte se construit dans RViz2.  
Sauvegarder la carte quand elle est complète :

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/ma_carte
```

---

## Navigation autonome (Nav2)

Nav2 est la stack de navigation ROS 2 standard. Elle prend en entrée une **position cible** et calcule elle-même la trajectoire en évitant les obstacles (LiDAR + costmap). Elle publie `/cmd_vel` vers le Pi — exactement comme le browser de téléopération.

```
Ton appli PC
    │  goal pose (action Nav2)
    ▼
Nav2 (PC) ──── /map · /scan · /tf ◄──── Pi
    │
    │  /cmd_vel
    ▼
yahboom_ctrl (Pi) → hardware
```

**PC — installer Nav2 (une fois)**
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

**Pi**
```bash
./docker/run_jazzy.sh nav /root/maps/ma_carte.yaml
```

**PC — lancer Nav2**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml

ros2 launch nav2_bringup navigation_launch.py \
  map:=/path/to/ma_carte.yaml \
  params_file:=/path/to/nav2_params.yaml
```

Depuis RViz2 : bouton **2D Nav Goal** → cliquer sur la carte → le robot y va seul.  
Depuis ton appli : publier sur `/move_base_simple/goal` (PoseStamped) ou utiliser l'action `NavigateToPose`.

> **Pourquoi Nav2 tourne sur le PC et pas le Pi ?**  
> Pour le développement c'est plus pratique (RAM, RViz2, rechargement rapide). Quand le robot devra fonctionner sans PC, Nav2 peut migrer dans le Docker Pi — le Pi 5 a la puissance nécessaire. C'est une étape future.

---

## Réseau PC ↔ Pi

Les deux machines doivent être sur le même LAN avec `ROS_DOMAIN_ID=0`.

```bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dogzilla/fastdds_unicast.xml
```

`fastdds_unicast.xml` désactive le multicast (obligatoire sur la plupart des réseaux Wi-Fi).  
Éditer `<address>` dans ce fichier pour y mettre l'IP statique du Pi si nécessaire.

---

## Construire et transférer l'image Docker

> À faire sur le PC à chaque modification du Dockerfile ou des packages ROS 2.  
> Cross-compilation x86 → ARM64 via QEMU + buildx.

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
ssh pi@<pi-ip> sudo docker load -i dogzilla_jazzy_arm64.tar
```

---

## Référence des nœuds ROS 2

### Pi — container Docker (toujours actifs)

| Nœud | Package | Rôle |
|---|---|---|
| `yahboom_ctrl` | `yahboom_base` | Traduit `/cmd_vel` + `/dogzilla/*` → DOGZILLALib → frames série `/dev/ttyAMA0` |
| `robot_state_publisher` | `robot_state_publisher` | URDF → `/tf_static` (squelette du robot) |
| `yahboomcar_joint_state` | `yahboom_dog_joint_state` | Angles servos + IMU → `/joint_states` + `/imu/data_raw_self` |
| `usb_cam_node_exe` | `usb_cam` | `/dev/video0` → `/image_raw` |
| `rosbridge_websocket` | `rosbridge_server` | WebSocket `:9090` — JSON roslibjs ↔ topics DDS |
| `dogzilla_web_server` | `dogzilla_teleop` | HTTP `:8080` — sert `teleop.html`, `watch.html`, `manifest.json` |

### Pi — container Docker (mode SLAM uniquement)

| Nœud | Package | Rôle |
|---|---|---|
| `slam_toolbox` | `slam_toolbox` | Fusionne `/scan` + `/tf` → construit et publie `/map` |
| driver LiDAR | `oradar_lidar` | Lit le LiDAR, publie `/scan` |

### PC (optionnel)

| Composant | Rôle |
|---|---|
| `rviz2` | Jumeau numérique — RobotModel, TF, LaserScan, Map, costmap, trajectoires |
| `nav2_bringup` | Navigation autonome — planificateur global + local, costmap ; envoie `/cmd_vel` au Pi |
| `slam_toolbox` (localisation) | Charge une carte existante et localise le robot dedans |

### Nœuds de perception (optionnels, à lancer manuellement)

| Nœud | Package | Rôle |
|---|---|---|
| `yahboom_color_tracking` | `yahboom_color_tracking` | Suit un objet coloré → publie `/cmd_vel` |
| `yahboom_qrcode_tracking` | `yahboom_qrcode_tracking` | Détecte et suit un QR code |
| `yahboom_mediapipe` | `yahboom_mediapipe` | Landmarks main/pose/visage → `/mediapipe/points` |
| `yahboom_publish` (C++) | `yahboom_publish` | Pipeline caméra → `/image_raw/compressed`, `/image_contours`, `/obj_msg` |
| Laser tracker / avoider | `yahboom_laser` | Évitement d'obstacles et suivi d'objet par LiDAR, buzzer de proximité |

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
| `/scan` | `sensor_msgs/LaserScan` | driver LiDAR | `slam_toolbox` · nœuds laser |
| `/joint_states` | `sensor_msgs/JointState` | `yahboomcar_joint_state` | `robot_state_publisher` |
| `/imu/data_raw_self` | `sensor_msgs/Imu` | `yahboomcar_joint_state` | Nav2 |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` (Pi) | Nav2 · RViz2 (PC) |
| `/tf` · `/tf_static` | — | `robot_state_publisher` · `tf_publisher` | RViz2 · Nav2 |

---

## Structure du dépôt

```
dogzilla/
├── DOGZILLALib/              bibliothèque hardware — framing série vers /dev/ttyAMA0
├── app_dogzilla/             app Flask mode Classic (ports 6500 HTTP · 6000 TCP)
├── docker/
│   ├── Dockerfile.jazzy      image Pi : ROS Jazzy + slam-toolbox + rosbridge + teleop
│   ├── entrypoint.sh         démarrage normal : yahboom_ctrl + capteurs + rosbridge + web
│   ├── entrypoint_slam.sh    démarrage SLAM : + slam_toolbox + lidar
│   └── run_jazzy.sh          lanceur — modes : robot (défaut) / slam / nav
├── samples/                  notebooks Jupyter (contrôle, vision, LLM)
├── yahboomcar_ws/src/
│   ├── dogzilla_teleop/      interface web — teleop.html + watch.html (servis par le Pi)
│   ├── yahboom_base/         bridge hardware — nœud yahboom_ctrl
│   ├── yahboom_bringup/      launch files SLAM + Nav2
│   ├── yahboom_description/  modèle URDF
│   └── …                     20+ packages ROS 2 (perception, laser, mediapipe…)
├── fastdds_unicast.xml       discovery DDS en unicast (Wi-Fi sans multicast)
└── CLAUDE.md                 guide pour l'assistant IA
```

---

<div align="center">
<sub>ROS 2 Jazzy · Yahboom Dogzilla S2 · Raspberry Pi 5</sub>
</div>

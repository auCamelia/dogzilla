#!/bin/bash
# Navigation autonome — Nav2 + stack robot complète sur le Pi.
# Construire une carte d'abord avec : ./run_jazzy.sh --slam
# Usage : ./run_jazzy.sh --nav [/root/maps/ma_carte.yaml]

if [ ! -f /root/yahboomcar_ws/install/setup.bash ]; then
    echo "ERROR: workspace non construit. Lancer d'abord : ./run_jazzy.sh --build"
    exit 1
fi

MAP_FILE="${MAP_FILE:-/root/maps/map.yaml}"
if [ ! -f "${MAP_FILE}" ]; then
    echo "ERROR: carte introuvable : ${MAP_FILE}"
    echo "Construire une carte avec : ./run_jazzy.sh --slam"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0

PARAMS_FILE="/root/yahboomcar_ws/src/yahboom_bringup/config/nav2_params.yaml"

echo "[NAV] Carte     : ${MAP_FILE}"
echo "[NAV] Params    : ${PARAMS_FILE}"

# Hardware bridge (cmd_vel → serial)
ros2 launch yahboom_base yahboom_base.launch.py &

# Robot description + joint states
ros2 launch yahboom_description yahboom_urdf.launch.py &
ros2 run yahboom_dog_joint_state yahboomcar_joint_state &

# Camera
ros2 run usb_cam usb_cam_node_exe &

# Teleop web (monitoring + override manuel possible pendant nav)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run dogzilla_teleop web_server --ros-args -p open_browser:=false &

# LiDAR OradarMS200 — publie /scan + TF base_link→laser_frame
ros2 launch oradar_lidar ms200_scan.launch.py &

# Odométrie laser — pas d'encodeurs sur ce robot.
# rf2o_laser_odometry estime /odom à partir de scans consécutifs.
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py \
  laser_scan_topic:=/scan \
  odom_topic:=/odom \
  base_frame_id:=base_footprint \
  odom_frame_id:=odom \
  freq:=10.0 &

# Nav2 — map server + AMCL + planner + controller + costmaps
ros2 launch nav2_bringup bringup_launch.py \
  map:="${MAP_FILE}" \
  params_file:="${PARAMS_FILE}" \
  use_sim_time:=false &

wait

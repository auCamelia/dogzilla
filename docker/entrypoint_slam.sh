#!/bin/bash
# SLAM cartography mode — runs on Pi
# Nav2 runs on PC (see dogzilla_nav package)
if [ ! -f /root/yahboomcar_ws/install/setup.bash ]; then
    echo "ERROR: workspace non construit. Lancer d'abord : ./run_jazzy.sh --build"
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONPATH=/root/DOGZILLALib:${PYTHONPATH}

mkdir -p /root/maps

echo "[SLAM] Starting cartography (slam_toolbox)"
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

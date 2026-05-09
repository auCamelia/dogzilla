#!/bin/bash
# SLAM cartography mode — runs on Pi
# Nav2 runs on PC (see dogzilla_nav package)
source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0

mkdir -p /root/maps

echo "[SLAM] Starting cartography (slam_toolbox)"
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

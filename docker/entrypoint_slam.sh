#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# Rebuild du package dogzilla_nav si nécessaire (Python only, rapide)
colcon build --packages-select dogzilla_nav --symlink-install \
  --base-paths /root/yahboomcar_ws \
  --build-base /root/yahboomcar_ws/build \
  --install-base /root/yahboomcar_ws/install \
  2>&1 | tail -5

source /root/yahboomcar_ws/install/setup.bash

mkdir -p /root/maps

if [ "${MODE}" = "nav" ]; then
  echo "[SLAM] Mode navigation — carte : ${MAP}"
  ros2 launch dogzilla_nav navigation.launch.py map:="${MAP}"
else
  echo "[SLAM] Mode cartographie"
  ros2 launch dogzilla_nav slam.launch.py
fi

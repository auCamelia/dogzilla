#!/bin/bash
# SLAM mode — full robot stack + slam_toolbox for map building.
# Drive the robot with the teleop UI while the map builds.
# Save the map with: ros2 run nav2_map_server map_saver_cli -f /root/maps/my_map
# Usage: ./run_jazzy.sh --slam

if [ ! -f /root/yahboomcar_ws/install/setup.bash ]; then
    echo "ERROR: workspace not built. Run first: ./run_jazzy.sh --build"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONPATH=/root/DOGZILLALib:${PYTHONPATH}

mkdir -p /root/maps

echo "[SLAM] Hardware bridge + LiDAR odometry (rf2o) + slam_toolbox"

# Hardware bridge: cmd_vel → serial, joint states + IMU at 10 Hz.
ros2 run yahboom_base ctrl &

# Robot description
ros2 launch yahboom_description yahboom_urdf.launch.py &

# Camera
ros2 run usb_cam usb_cam_node_exe &

# Teleop web UI
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run dogzilla_teleop web_server --ros-args -p open_browser:=false &

# LiDAR OradarMS200 — publishes /scan + TF base_link→laser_frame
ros2 launch oradar_lidar ms200_scan.launch.py &

# LiDAR odometry — scan matching between consecutive scans.
# Use ros2 run (not launch) to actually apply parameter overrides —
# rf2o_laser_odometry.launch.py hardcodes params without LaunchConfiguration.
# Use --params-file so init_pose_from_topic="" parses correctly (rcl rejects bare -p x:=).
RF2O_PARAMS="/root/yahboomcar_ws/src/yahboom_bringup/config/rf2o_params.yaml"
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  --params-file "${RF2O_PARAMS}" \
  -p publish_tf:=true &

# SLAM — builds and publishes /map from /scan + odometry
SLAM_PARAMS="/root/yahboomcar_ws/src/yahboom_bringup/config/slam_toolbox_params.yaml"
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:="${SLAM_PARAMS}" &

wait

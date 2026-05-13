#!/bin/bash
# Robot mode — teleop + LiDAR odometry (rf2o) + IMU fusion via EKF.
# No Nav2. Ideal for manual driving with digital twin in RViz2.
# Usage: ./run_jazzy.sh --robot

if [ ! -f /root/yahboomcar_ws/install/setup.bash ]; then
    echo "ERROR: workspace not built. Run first: ./run_jazzy.sh --build"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONPATH=/root/DOGZILLALib:${PYTHONPATH}

PARAMS_FILE="/root/yahboomcar_ws/src/yahboom_bringup/config/ekf_robot.yaml"

echo "[ROBOT] Hardware bridge + LiDAR odometry (rf2o)"
echo "[ROBOT] IMU fusion via EKF (robot_localization)"
echo "[ROBOT] Params: ${PARAMS_FILE}"

# Hardware bridge: cmd_vel → serial, joint states + IMU at 10 Hz.
# No odom publication — rf2o owns /odom. No TF — EKF owns odom→base_link.
ros2 run yahboom_base ctrl &

# Robot description
ros2 launch yahboom_description yahboom_urdf.launch.py &

# Camera
ros2 run usb_cam usb_cam_node_exe &

# Teleop web UI
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run dogzilla_teleop web_server --ros-args -p open_browser:=false &

# LiDAR OradarMS200 — publie /scan + TF base_link→laser_frame
ros2 launch oradar_lidar ms200_scan.launch.py &

# LiDAR odometry — scan matching between consecutive scans.
# publish_tf:=false : EKF owns the odom→base_link TF.
# Use ros2 run (not launch) — rf2o launch file hardcodes params without LaunchConfiguration.
# Use --params-file so init_pose_from_topic="" parses correctly (rcl rejects bare -p x:=).
RF2O_PARAMS="/root/yahboomcar_ws/src/yahboom_bringup/config/rf2o_params.yaml"
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  --params-file "${RF2O_PARAMS}" \
  -p publish_tf:=false &

# EKF: fuses /odom (rf2o scan matching) + /imu/data_raw_self (roll/pitch/yaw)
# → publishes /odometry/filtered + TF odom→base_link with full 3D orientation.
ros2 run robot_localization ekf_node --ros-args \
  --params-file "${PARAMS_FILE}" &

wait

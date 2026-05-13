#!/bin/bash
# Nav mode — same base stack as --robot + Nav2 (AMCL + planner + costmaps).
# Build a map first with: ./run_jazzy.sh --slam
# Usage: ./run_jazzy.sh --nav [/root/maps/my_map.yaml]

if [ ! -f /root/yahboomcar_ws/install/setup.bash ]; then
    echo "ERROR: workspace not built. Run first: ./run_jazzy.sh --build"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONPATH=/root/DOGZILLALib:${PYTHONPATH}

EKF_PARAMS="/root/yahboomcar_ws/src/yahboom_bringup/config/ekf_robot.yaml"
NAV2_PARAMS="/root/yahboomcar_ws/src/yahboom_bringup/config/nav2_params.yaml"

if [ -n "${MAP_FILE}" ] && [ -f "${MAP_FILE}" ]; then
    echo "[NAV] Map       : ${MAP_FILE}"
    echo "[NAV] Nav2 params: ${NAV2_PARAMS}"
    WITH_NAV2=true
else
    echo "[NAV] No map — teleop + odometry only (no Nav2)"
    WITH_NAV2=false
fi
echo "[NAV] EKF params: ${EKF_PARAMS}"

# Hardware bridge: cmd_vel → serial, joint states + IMU at 10 Hz.
# No odom publication — rf2o owns /odom. No TF — EKF owns odom→base_link.
ros2 run yahboom_base ctrl &

# Robot description
ros2 launch yahboom_description yahboom_urdf.launch.py &

# Camera
ros2 run usb_cam usb_cam_node_exe &

# Teleop web (monitoring + manual override possible during nav)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run dogzilla_teleop web_server --ros-args -p open_browser:=false &

# LiDAR OradarMS200 — publishes /scan + TF base_link→laser_frame
ros2 launch oradar_lidar ms200_scan.launch.py &

# LiDAR odometry — scan matching between consecutive scans.
# publish_tf:=false : EKF owns the odom→base_link TF.
# Use ros2 run (not launch) — rf2o launch file hardcodes params without LaunchConfiguration.
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p publish_tf:=false \
  -p freq:=10.0 \
  -p "init_pose_from_topic:=" &

# EKF: fuses /odom (rf2o scan matching) + /imu/data_raw_self (roll/pitch/yaw)
# → publishes /odometry/filtered + TF odom→base_link with full 3D orientation.
ros2 run robot_localization ekf_node --ros-args \
  --params-file "${EKF_PARAMS}" &

# Nav2 — map server + AMCL + planner + controller + costmaps (if map available)
if [ "${WITH_NAV2}" = true ]; then
    ros2 launch nav2_bringup bringup_launch.py \
      map:="${MAP_FILE}" \
      params_file:="${NAV2_PARAMS}" \
      use_sim_time:=false &
fi

wait

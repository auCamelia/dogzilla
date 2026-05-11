#!/bin/bash
# Robot mode — teleop + dead-reckoning + IMU fusion via EKF.
# No LiDAR, no Nav2. Ideal for manual driving with digital twin in RViz2.
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

echo "[ROBOT] Hardware bridge + dead-reckoning odom"
echo "[ROBOT] IMU fusion via EKF (robot_localization)"
echo "[ROBOT] Params: ${PARAMS_FILE}"

# Hardware bridge: cmd_vel → serial, publishes /odom (dead-reckoning) + TF odom→base_footprint
ros2 run yahboom_base ctrl --ros-args -p publish_odom:=true &

# Robot description + joint states + IMU
ros2 launch yahboom_description yahboom_urdf.launch.py &
ros2 run yahboom_dog_joint_state yahboomcar_joint_state &

# Camera
ros2 run usb_cam usb_cam_node_exe &

# Teleop web UI
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run dogzilla_teleop web_server --ros-args -p open_browser:=false &

# EKF: fuses /odom (vx,vy,vyaw) + /imu/data_raw_self (roll,pitch,yaw orientation)
ros2 run robot_localization ekf_node --ros-args \
  --params-file "${PARAMS_FILE}" &

wait

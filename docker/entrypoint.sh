#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch oradar_lidar ms200_scan.launch.py &
ros2 launch yahboom_description yahboom_urdf.launch.py &
ros2 run usb_cam usb_cam_node_exe &
ros2 run yahboom_dog_joint_state yahboomcar_joint_state &

wait

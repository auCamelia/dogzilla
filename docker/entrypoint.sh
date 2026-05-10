#!/bin/bash
if [ ! -f /root/yahboomcar_ws/install/setup.bash ]; then
    echo "ERROR: workspace non construit. Lancer d'abord : ./run_jazzy.sh --build"
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# Hardware bridge (cmd_vel → serial)
ros2 launch yahboom_base yahboom_base.launch.py &

# Robot description + joint states
ros2 launch yahboom_description yahboom_urdf.launch.py &
ros2 run yahboom_dog_joint_state yahboomcar_joint_state &

# Camera
ros2 run usb_cam usb_cam_node_exe &

# Teleop web interface — rosbridge :9090, web server :8080
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run dogzilla_teleop web_server --ros-args -p open_browser:=false &

wait

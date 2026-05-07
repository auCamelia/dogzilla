#!/bin/bash
xhost +
docker run -it \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="ROS_DOMAIN_ID=0" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/pi/yahboomcar_ws/src:/root/yahboomcar_ws/src \
--device=/dev/ttyAMA1 \
--device=/dev/ttyAMA0 \
--device=/dev/video0 \
--device=/dev/input \
dogzilla:jazzy

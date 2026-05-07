#!/bin/bash
xhost +
docker run -it \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_unicast.xml" \
--env="ROS_DOMAIN_ID=0" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/pi/fastdds_unicast.xml:/fastdds_unicast.xml \
--device=/dev/ttyAMA1 \
--device=/dev/ttyAMA0 \
--device=/dev/video0 \
--device=/dev/input \
yahboomtechnology/ros-humble:3.6 /bin/bash 


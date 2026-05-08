#!/bin/bash
# Lance le container SLAM (dogzilla:slam)
# Modes : slam (cartographie) | nav (navigation avec carte existante)
# Usage : ./run_slam.sh [slam|nav] [chemin_carte.yaml]

MODE=${1:-slam}
MAP=${2:-/root/maps/map.yaml}

xhost +

docker run -it \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="ROS_DOMAIN_ID=0" \
  --env="MODE=${MODE}" \
  --env="MAP=${MAP}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/pi/yahboomcar_ws/src:/root/yahboomcar_ws/src \
  -v /home/pi/maps:/root/maps \
  --device=/dev/ttyAMA1 \
  --device=/dev/ttyAMA0 \
  --device=/dev/video0 \
  --device=/dev/input \
  --entrypoint /entrypoint_slam.sh \
  dogzilla:slam

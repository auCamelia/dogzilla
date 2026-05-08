#!/bin/bash
# Usage: ./run_jazzy.sh [slam|nav] [chemin_carte.yaml]
# Par défaut : mode robot (entrypoint normal)
MODE=${1:-robot}
MAP=${2:-/root/maps/map.yaml}

xhost +

ENTRYPOINT_ARG=""
if [ "${MODE}" = "slam" ] || [ "${MODE}" = "nav" ]; then
    ENTRYPOINT_ARG="--entrypoint /entrypoint_slam.sh"
fi

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
  ${ENTRYPOINT_ARG} \
  dogzilla:jazzy

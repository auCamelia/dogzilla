#!/bin/bash
# Usage:
#   ./run_jazzy.sh          — robot mode (hardware bridge)
#   ./run_jazzy.sh slam     — SLAM cartography (slam_toolbox)
#
# Navigation (Nav2) runs on the PC — see dogzilla_nav package.

MODE=${1:-robot}

ENTRYPOINT_ARG=""
if [ "${MODE}" = "slam" ]; then
    ENTRYPOINT_ARG="--entrypoint /entrypoint_slam.sh"
fi

docker run -it \
  --net=host \
  --env="ROS_DOMAIN_ID=0" \
  --env="MODE=${MODE}" \
  -v /home/pi/yahboomcar_ws/src:/root/yahboomcar_ws/src \
  -v /home/pi/maps:/root/maps \
  --device=/dev/ttyAMA1 \
  --device=/dev/ttyAMA0 \
  --device=/dev/video0 \
  --device=/dev/input \
  ${ENTRYPOINT_ARG} \
  dogzilla:jazzy

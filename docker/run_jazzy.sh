#!/bin/bash
# Usage:
#   ./run_jazzy.sh [--robot]              — robot mode, default
#   ./run_jazzy.sh --slam                 — SLAM cartography (slam_toolbox)
#   ./run_jazzy.sh --nav [map.yaml]       — navigation autonome Nav2
#                                           map.yaml = chemin container (/root/maps/…)
#                                           défaut : /root/maps/map.yaml
#   ./run_jazzy.sh --build                — colcon build → install/ log/ build/ persistés sur Pi
#
# Le workspace (src/ install/ build/ log/) est monté depuis le repo Pi.
# Workflow initial :
#   git clone <repo> ~/dogzilla
#   ./docker/run_jazzy.sh --build         ← construit le workspace une fois
#   ./docker/run_jazzy.sh --robot         ← lance le robot

MODE="robot"
MAP_FILE="/root/maps/map.yaml"

i=1
while [ $i -le $# ]; do
  arg="${!i}"
  case "$arg" in
    --robot) MODE="robot" ;;
    --slam)  MODE="slam"  ;;
    --build) MODE="build" ;;
    --nav)
      MODE="nav"
      i=$((i + 1))
      [ $i -le $# ] && MAP_FILE="${!i}"
      ;;
    *) echo "Usage: $0 [--robot|--slam|--build|--nav [map.yaml]]" >&2; exit 1 ;;
  esac
  i=$((i + 1))
done

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORKSPACE="${REPO_ROOT}/yahboomcar_ws"

ENTRYPOINT_ARG=""
case "$MODE" in
  slam)  ENTRYPOINT_ARG="--entrypoint /entrypoint_slam.sh" ;;
  build) ENTRYPOINT_ARG="--entrypoint /entrypoint_build.sh" ;;
  nav)   ENTRYPOINT_ARG="--entrypoint /entrypoint_nav.sh" ;;
esac

docker run -it \
  --net=host \
  --env="ROS_DOMAIN_ID=0" \
  --env="MODE=${MODE}" \
  --env="MAP_FILE=${MAP_FILE}" \
  -v "${WORKSPACE}:/root/yahboomcar_ws" \
  -v /home/pi/maps:/root/maps \
  --device=/dev/ttyAMA1 \
  --device=/dev/ttyAMA0 \
  --device=/dev/video0 \
  --device=/dev/input \
  ${ENTRYPOINT_ARG} \
  dogzilla:jazzy

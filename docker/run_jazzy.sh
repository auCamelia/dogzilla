#!/bin/bash
# Usage:
#   ./run_jazzy.sh                        — nav mode (default): teleop + LiDAR + rf2o, no Nav2
#   ./run_jazzy.sh --nav [map.yaml]       — nav mode + Nav2 autonomous navigation
#                                           map.yaml = container path (/root/maps/…)
#                                           default: /root/maps/map.yaml
#   ./run_jazzy.sh --robot                — robot mode: teleop + dead-reckoning + IMU/EKF, no LiDAR
#   ./run_jazzy.sh --slam                 — SLAM cartography (slam_toolbox)
#   ./run_jazzy.sh --build                — colcon build (workspace persisted on Pi)
#
# Workspace (src/ install/ build/ log/) is volume-mounted from the Pi repo.
# Initial workflow:
#   git clone <repo> ~/dogzilla
#   ./docker/run_jazzy.sh --build         ← build once
#   ./docker/run_jazzy.sh                 ← run the robot

MODE="nav"
MAP_FILE=""

i=1
while [ $i -le $# ]; do
  arg="${!i}"
  case "$arg" in
    --slam)   MODE="slam"   ;;
    --build)  MODE="build"  ;;
    --robot)  MODE="robot"  ;;
    --nav)
      MODE="nav"
      i=$((i + 1))
      [ $i -le $# ] && [[ "${!i}" != --* ]] && MAP_FILE="${!i}" || true
      ;;
    *) echo "Usage: $0 [--nav [map.yaml]|--robot|--slam|--build]" >&2; exit 1 ;;
  esac
  i=$((i + 1))
done

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORKSPACE="${REPO_ROOT}/yahboomcar_ws"

case "$MODE" in
  nav)    ENTRYPOINT="/docker/entrypoint_nav.sh"   ;;
  slam)   ENTRYPOINT="/docker/entrypoint_slam.sh"  ;;
  build)  ENTRYPOINT="/docker/entrypoint_build.sh" ;;
  robot)  ENTRYPOINT="/docker/entrypoint_robot.sh" ;;
esac

docker run -it --rm \
  --net=host \
  --env="ROS_DOMAIN_ID=0" \
  --env="MAP_FILE=${MAP_FILE}" \
  -v "${WORKSPACE}:/root/yahboomcar_ws" \
  -v "${REPO_ROOT}/docker:/docker:ro" \
  -v "${REPO_ROOT}/DOGZILLALib:/root/DOGZILLALib:ro" \
  -v /home/pi/maps:/root/maps \
  --device=/dev/ttyAMA1 \
  --device=/dev/ttyAMA0 \
  --device=/dev/video0 \
  --device=/dev/input \
  --entrypoint "${ENTRYPOINT}" \
  dogzilla:jazzy

#!/bin/bash
# Build mode — colcon build complet, résultats persistés sur le Pi via volume mount.
# Lance avec : ./run_jazzy.sh --build

source /opt/ros/jazzy/setup.bash

cd /root/yahboomcar_ws
mkdir -p log

# Incremental build si install/ existe déjà
[ -f install/setup.bash ] && source install/setup.bash

echo "[BUILD] colcon build — $(date)"
# Tout est compilé ici car on est en natif ARM64 sur le Pi.
# Dockerfile skippait oradar_lidar/ydlidar pour la cross-compilation x86→ARM.
colcon build \
  --symlink-install \
  --parallel-workers 2 \
  2>&1 | tee log/colcon_build.log

STATUS=${PIPESTATUS[0]}
echo "[BUILD] Terminé (exit ${STATUS}) — install/ log/ build/ mis à jour sur le Pi."
exit "${STATUS}"

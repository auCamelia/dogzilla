#!/bin/bash
# Build mode — colcon build complet, résultats persistés sur le Pi via volume mount.
# Lance avec : ./run_jazzy.sh --build

source /opt/ros/jazzy/setup.bash

cd /root/yahboomcar_ws
mkdir -p log src

# rf2o_laser_odometry — not available as apt package for Jazzy, build from source
RF2O_DIR="src/rf2o_laser_odometry"
if [ ! -d "${RF2O_DIR}" ]; then
    echo "[BUILD] Cloning rf2o_laser_odometry..."
    git clone --depth 1 https://github.com/MAPIRlab/rf2o_laser_odometry.git "${RF2O_DIR}"
fi

# Incremental build si install/ existe déjà
[ -f install/setup.bash ] && source install/setup.bash

echo "[BUILD] colcon build — $(date)"
colcon build \
  --symlink-install \
  --parallel-workers 2 \
  2>&1 | tee log/colcon_build.log

STATUS=${PIPESTATUS[0]}
echo "[BUILD] Terminé (exit ${STATUS}) — install/ log/ build/ mis à jour sur le Pi."
exit "${STATUS}"

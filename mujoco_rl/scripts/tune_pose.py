#!/usr/bin/env python3
"""Open the full interactive MuJoCo GUI (not the minimal passive viewer) so you
can drag each joint's control slider live and find the correct starting pose
by eye, comparing directly against the real robot -- no need to relaunch a
script for every angle guess.

Gravity is disabled in this script -- the robot floats in place instead of
falling/tipping over, so you can freely pose the legs with the sliders no
matter the angle, without needing to prop it up or worry about balance.

In the window that opens:
  - Open the "Control" panel (right-hand sidebar, or press Tab if hidden) to
    see one slider per actuator: lf_hip, lf_upper, lf_lower, rf_hip, ... etc.
    Dragging a slider sets that joint's target angle directly.
  - Angles shown are in RADIANS (MuJoCo's native unit), not degrees. To
    convert back to DOGZILLALib.motor()-style degrees: degrees = radians * 180/pi.
  - Close the window when done -- read off the final joint angles printed to
    the terminal.

Usage:
    python3 scripts/tune_pose.py
"""
import pathlib

import mujoco
import mujoco.viewer
import numpy as np

MODEL_PATH = pathlib.Path(__file__).resolve().parent.parent / "models" / "dogzilla.xml"

JOINT_NAMES = [
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",
]


# The MJCF's joint ranges/ctrlranges come from DOGZILLALib.PARAM["MOTOR_LIMIT"],
# which turned out to be too narrow for at least the lower-leg joint (real
# mechanical range confirmed >90 degrees, vs. the +-31deg used there) -- widen
# everything generously here so the sliders never clip while you explore,
# without touching the real training model's (still-unresolved) ranges.
EXPLORATION_RANGE_DEG = 170.0


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0  # float in place -- pose the legs without the body falling/tipping

    exploration_range_rad = np.deg2rad(EXPLORATION_RANGE_DEG)
    joint_ids = np.arange(1, 13)  # joint 0 is the base freejoint, skip it
    model.jnt_range[joint_ids] = [-exploration_range_rad, exploration_range_rad]
    model.actuator_ctrlrange[:] = [-exploration_range_rad, exploration_range_rad]

    data.qpos[2] = 0.15
    data.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)

    print("Opening the full MuJoCo GUI -- use the 'Control' sliders to pose the robot.")
    print("Close the window when you're happy with a pose; the final joint angles print below.\n")

    mujoco.viewer.launch(model, data)  # blocks until the window is closed

    print("\nFinal joint angles when the window was closed:")
    joint_qpos_adr = model.jnt_qposadr[np.arange(1, 13)]
    final_rad = data.qpos[joint_qpos_adr]
    for name, rad in zip(JOINT_NAMES, final_rad):
        print(f"  {name:20s} {rad:+.4f} rad  = {np.rad2deg(rad):+7.2f} deg")


if __name__ == "__main__":
    main()

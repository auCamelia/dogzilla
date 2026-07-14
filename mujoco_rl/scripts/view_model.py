#!/usr/bin/env python3
"""Sanity-check viewer for models/dogzilla.xml.

Drops the robot from a small height with all joints at their zero position and
lets it settle under gravity, so you can visually confirm proportions, mass,
and standing stability before writing/training against the Gymnasium env.

Usage:
    python3 scripts/view_model.py            # interactive viewer
    python3 scripts/view_model.py --headless  # print settle diagnostics only
"""
import argparse
import pathlib

import mujoco
import numpy as np

MODEL_PATH = pathlib.Path(__file__).resolve().parent.parent / "models" / "dogzilla.xml"


def load_model():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    return model, data


def report_settle(model, data, seconds=3.0):
    mujoco.mj_resetData(model, data)
    steps = int(seconds / model.opt.timestep)
    for _ in range(steps):
        mujoco.mj_step(model, data)

    base_z = data.qpos[2]
    base_quat = data.qpos[3:7]
    print(f"Settled after {seconds:.1f}s simulated time:")
    print(f"  base_link height: {base_z:.4f} m")
    print(f"  base_link quat (w,x,y,z): {np.round(base_quat, 4)}")
    print(f"  base linear velocity: {np.round(data.qvel[:3], 4)}")
    if base_z < 0.05:
        print("  WARNING: base height very low — robot likely collapsed.")
    if abs(base_quat[0]) < 0.9:
        print("  WARNING: base orientation far from upright.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="skip the interactive viewer, just print settle diagnostics")
    args = parser.parse_args()

    model, data = load_model()
    report_settle(model, data)

    if args.headless:
        return

    import mujoco.viewer

    mujoco.mj_resetData(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()

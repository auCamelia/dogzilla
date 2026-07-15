#!/usr/bin/env python3
"""Set the robot to a specific candidate joint pose (in DOGZILLALib motor
degrees, same convention as `motor(motor_id, angle_deg)`) and inspect it --
either headless (renders a PNG, and optionally lets it settle under gravity
for a few seconds first) or in the interactive viewer.

This exists to verify a candidate *starting pose* for training episodes
against the real robot's actual power-on stance before baking it into
envs/dogzilla_env.py's reset() -- see mujoco_rl/README.md.

Usage:
    python3 scripts/check_pose.py                                  # hip=0, upper=45, lower=45 (same sign), renders pose_check.png
    python3 scripts/check_pose.py --lower-deg -45                  # opposite-sign knee bend, for comparison
    python3 scripts/check_pose.py --settle                          # also simulates 2s under gravity holding this pose, then renders
    python3 scripts/check_pose.py --interactive                     # opens the live viewer instead of rendering a PNG
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import mujoco
import mujoco.viewer
import numpy as np

MODEL_PATH = pathlib.Path(__file__).resolve().parent.parent / "models" / "dogzilla.xml"

HIP_JOINT_INDICES = [0, 3, 6, 9]
UPPER_JOINT_INDICES = [1, 4, 7, 10]
LOWER_JOINT_INDICES = [2, 5, 8, 11]


def set_pose(data, joint_qpos_adr, hip_deg, upper_deg, lower_deg):
    angles_deg = np.zeros(12)
    angles_deg[HIP_JOINT_INDICES] = hip_deg
    angles_deg[UPPER_JOINT_INDICES] = upper_deg
    angles_deg[LOWER_JOINT_INDICES] = lower_deg
    data.qpos[joint_qpos_adr] = np.deg2rad(angles_deg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hip-deg", type=float, default=0.0, help="hip (x1) motor angle, degrees -- 0 = parallel to ground per the real robot's power-on pose")
    parser.add_argument("--upper-deg", type=float, default=45.0, help="upper-leg (x2) motor angle, degrees")
    parser.add_argument("--lower-deg", type=float, default=45.0, help="lower-leg (x3) motor angle, degrees -- try -45 for the opposite-sign knee bend if +45 looks wrong")
    parser.add_argument("--settle", action="store_true", help="hold this pose with the position actuators and simulate 2s under gravity before rendering, instead of a raw kinematic snapshot")
    parser.add_argument("--interactive", action="store_true", help="open the live interactive viewer instead of rendering a PNG")
    parser.add_argument("--out", type=str, default="pose_check.png")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    joint_ids = np.arange(1, 13)
    joint_qpos_adr = model.jnt_qposadr[joint_ids]

    data.qpos[2] = 0.15
    data.qpos[3:7] = [1, 0, 0, 0]
    set_pose(data, joint_qpos_adr, args.hip_deg, args.upper_deg, args.lower_deg)

    if args.settle:
        data.ctrl[:] = np.deg2rad([
            args.hip_deg, args.upper_deg, args.lower_deg,
            args.hip_deg, args.upper_deg, args.lower_deg,
            args.hip_deg, args.upper_deg, args.lower_deg,
            args.hip_deg, args.upper_deg, args.lower_deg,
        ])
        mujoco.mj_forward(model, data)
        for _ in range(int(2.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
    else:
        mujoco.mj_forward(model, data)

    print(f"hip={args.hip_deg} upper={args.upper_deg} lower={args.lower_deg} deg  "
          f"(settle={args.settle})  base height={data.qpos[2]:.4f}m")

    if args.interactive:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                viewer.sync()
    else:
        renderer = mujoco.Renderer(model, height=480, width=640)
        renderer.update_scene(data, camera=-1)
        img = renderer.render()
        import imageio.v3 as iio
        iio.imwrite(args.out, img)
        print(f"Saved {args.out}")


if __name__ == "__main__":
    main()

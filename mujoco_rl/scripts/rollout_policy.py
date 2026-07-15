#!/usr/bin/env python3
"""Roll out a trained PPO checkpoint, either in the interactive MuJoCo viewer
or (headless, no display needed) recorded to an MP4 for offline review.

Usage:
    python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip
    python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip --no-randomize
    python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip --episodes 5
    python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip --record rollout.mp4 --no-randomize
    python3 scripts/rollout_policy.py checkpoints/stairs/final_model.zip --env stairs --record stairs.mp4 --no-randomize
    python3 scripts/rollout_policy.py checkpoints/stairs/final_model.zip --env stairs --no-randomize --speed 0.4  # slow motion
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import mujoco
import numpy as np
from stable_baselines3 import PPO

from envs.dogzilla_env import DogzillaWalkEnv
from envs.dogzilla_stairs_env import DogzillaStairsEnv

ENVS = {"walk": DogzillaWalkEnv, "stairs": DogzillaStairsEnv}


def run_interactive(model, env, episodes, seed, speed=1.0):
    import time

    import mujoco.viewer

    control_dt = env.model.opt.timestep * env.frame_skip

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        for ep in range(episodes):
            if not viewer.is_running():
                break
            obs, info = env.reset(seed=seed + ep)
            print(f"episode {ep}: cmd (vx, vy, vyaw) = {np.round(env._cmd, 3)}")
            episode_reward = 0.0
            while viewer.is_running():
                step_start = time.time()
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                episode_reward += reward
                viewer.sync()
                # Real-time pacing: without this, env.step() (10 physics
                # substeps = 20ms of simulated time) runs as fast as the CPU
                # allows, so 10 episodes fly by in a couple seconds.
                time_to_wait = control_dt / speed - (time.time() - step_start)
                if time_to_wait > 0:
                    time.sleep(time_to_wait)
                if terminated or truncated:
                    print(f"  ended after {env._step_count} steps, reward={episode_reward:.1f}, fell={terminated}")
                    break


def run_record(model, env, episodes, seed, out_path, fps=30):
    import imageio

    renderer = mujoco.Renderer(env.model, height=480, width=640)
    render_every = max(1, round(1.0 / (fps * env.model.opt.timestep * env.frame_skip)))

    with imageio.get_writer(out_path, fps=fps) as writer:
        for ep in range(episodes):
            obs, info = env.reset(seed=seed + ep)
            print(f"episode {ep}: cmd (vx, vy, vyaw) = {np.round(env._cmd, 3)}")
            episode_reward = 0.0
            step = 0
            while True:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                episode_reward += reward
                if step % render_every == 0:
                    renderer.update_scene(env.data, camera=-1)
                    writer.append_data(renderer.render())
                step += 1
                if terminated or truncated:
                    print(f"  ended after {env._step_count} steps, reward={episode_reward:.1f}, fell={terminated}")
                    break
    print(f"Saved rollout video to {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=str, help="path to a saved SB3 .zip checkpoint")
    parser.add_argument("--no-randomize", action="store_true", help="disable domain randomization for a clean nominal-physics rollout")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--record", type=str, default=None, help="if set, render headless to this MP4 path instead of opening the interactive viewer")
    parser.add_argument("--env", type=str, default="walk", choices=sorted(ENVS), help="which task env the checkpoint was trained on")
    parser.add_argument("--speed", type=float, default=1.0, help="interactive viewer playback speed multiplier (e.g. 0.3 to slow down, 2.0 to speed up)")
    args = parser.parse_args()

    model = PPO.load(args.checkpoint)
    env = ENVS[args.env](randomize=not args.no_randomize)

    if args.record:
        run_record(model, env, args.episodes, args.seed, args.record)
    else:
        run_interactive(model, env, args.episodes, args.seed, speed=args.speed)


if __name__ == "__main__":
    main()

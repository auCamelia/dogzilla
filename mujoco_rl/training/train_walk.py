#!/usr/bin/env python3
"""PPO training entry point for the DOGZILLA walking task.

Usage:
    python3 training/train_walk.py --smoke-test     # ~100k steps, a few minutes, sanity check only
    python3 training/train_walk.py --timesteps 20000000 --n-envs 8
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from envs.dogzilla_env import DogzillaWalkEnv

CHECKPOINT_DIR = pathlib.Path(__file__).resolve().parent.parent / "checkpoints"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-test", action="store_true", help="short run (~100k steps) to verify the training loop end-to-end")
    parser.add_argument("--timesteps", type=int, default=20_000_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--checkpoint-freq", type=int, default=200_000)
    parser.add_argument("--run-name", type=str, default="walk")
    args = parser.parse_args()

    timesteps = 100_000 if args.smoke_test else args.timesteps
    n_envs = 2 if args.smoke_test else args.n_envs

    vec_env = make_vec_env(DogzillaWalkEnv, n_envs=n_envs, vec_env_cls=SubprocVecEnv)

    model = PPO(
        "MlpPolicy",
        vec_env,
        verbose=1,
        n_steps=1024,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        learning_rate=3e-4,
    )

    run_dir = CHECKPOINT_DIR / args.run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.checkpoint_freq // n_envs, 1),
        save_path=str(run_dir),
        name_prefix="ppo_dogzilla",
    )

    model.learn(total_timesteps=timesteps, callback=checkpoint_callback, progress_bar=True)
    model.save(str(run_dir / "final_model"))
    print(f"Saved final model to {run_dir / 'final_model'}.zip")


if __name__ == "__main__":
    main()

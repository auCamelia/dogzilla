#!/usr/bin/env python3
"""PPO training entry point for the DOGZILLA plateau-crossing task (step up,
walk across, step down).

Usage:
    python3 training/train_plateau.py --smoke-test     # ~100k steps, a few minutes, sanity check only
    python3 training/train_plateau.py --timesteps 6000000 --n-envs 8 --warm-start checkpoints/walk/final_model.zip
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from envs.dogzilla_plateau_env import PLATEAU_HEIGHT_RANGE, DogzillaPlateauEnv

CHECKPOINT_DIR = pathlib.Path(__file__).resolve().parent.parent / "checkpoints"

CURRICULUM_START_HEIGHT_RANGE = (0.0, 0.005)  # near-flat, to first nail the approach + a trivial bump
CURRICULUM_RAMP_FRACTION = 0.7  # reach full difficulty by 70% through training, refine after


class PlateauCurriculum(BaseCallback):
    """Linearly widens the plateau-height range from CURRICULUM_START_HEIGHT_RANGE
    to PLATEAU_HEIGHT_RANGE over the first `ramp_fraction` of training -- see
    envs/dogzilla_stairs_env.py's StairsCurriculum for the same idea applied
    to the (harder, multi-step) stairs task, and mujoco_rl/README.md for why a
    fixed full-difficulty terrain from step zero doesn't work well."""

    def __init__(self, total_timesteps, end_height_range=PLATEAU_HEIGHT_RANGE,
                 start_height_range=CURRICULUM_START_HEIGHT_RANGE,
                 ramp_fraction=CURRICULUM_RAMP_FRACTION, update_freq=20_000, verbose=0):
        super().__init__(verbose)
        self.total_timesteps = total_timesteps
        self.start_height_range = start_height_range
        self.end_height_range = end_height_range
        self.ramp_fraction = ramp_fraction
        self.update_freq = update_freq
        self._last_update = -update_freq

    def _on_step(self):
        if self.num_timesteps - self._last_update < self.update_freq:
            return True
        self._last_update = self.num_timesteps
        t = min(1.0, (self.num_timesteps / self.total_timesteps) / self.ramp_fraction)
        low = self.start_height_range[0] + t * (self.end_height_range[0] - self.start_height_range[0])
        high = self.start_height_range[1] + t * (self.end_height_range[1] - self.start_height_range[1])
        self.training_env.env_method("set_plateau_height_range", low, high)
        if self.verbose:
            print(f"[curriculum] progress={t:.2f} plateau_height_range=({low:.4f}, {high:.4f})")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-test", action="store_true", help="short run (~100k steps) to verify the training loop end-to-end")
    parser.add_argument("--timesteps", type=int, default=20_000_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--checkpoint-freq", type=int, default=200_000)
    parser.add_argument("--run-name", type=str, default="plateau")
    parser.add_argument("--warm-start", type=str, default=None, help="path to a checkpoint (e.g. the flat-ground walk policy) to initialize weights from")
    parser.add_argument("--no-curriculum", action="store_true", help="disable the plateau-height curriculum, use the full PLATEAU_HEIGHT_RANGE from step zero")
    parser.add_argument("--learning-rate", type=float, default=3e-4, help="lower this (e.g. 5e-5) when continuing training from a checkpoint that already works")
    args = parser.parse_args()

    timesteps = 100_000 if args.smoke_test else args.timesteps
    n_envs = 2 if args.smoke_test else args.n_envs

    vec_env = make_vec_env(DogzillaPlateauEnv, n_envs=n_envs, vec_env_cls=SubprocVecEnv)

    if args.warm_start:
        model = PPO.load(args.warm_start, env=vec_env, learning_rate=args.learning_rate)
    else:
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
        name_prefix="ppo_dogzilla_plateau",
    )

    callbacks = [checkpoint_callback]
    if not args.no_curriculum:
        callbacks.append(PlateauCurriculum(total_timesteps=timesteps, verbose=1))

    model.learn(total_timesteps=timesteps, callback=CallbackList(callbacks), progress_bar=True)
    model.save(str(run_dir / "final_model"))
    print(f"Saved final model to {run_dir / 'final_model'}.zip")


if __name__ == "__main__":
    main()

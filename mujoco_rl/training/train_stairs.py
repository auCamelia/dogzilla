#!/usr/bin/env python3
"""PPO training entry point for the DOGZILLA stair-climbing task.

Usage:
    python3 training/train_stairs.py --smoke-test     # ~100k steps, a few minutes, sanity check only
    python3 training/train_stairs.py --timesteps 5000000 --n-envs 8
"""
import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from envs.dogzilla_stairs_env import STEP_HEIGHT_RANGE, DogzillaStairsEnv
from envs.terrain import NUM_STEPS

CHECKPOINT_DIR = pathlib.Path(__file__).resolve().parent.parent / "checkpoints"

CURRICULUM_START_HEIGHT_RANGE = (0.0, 0.005)  # near-flat, to first nail the approach + a trivial bump
CURRICULUM_START_ACTIVE_STEPS = 1  # solve one riser at a time before chaining all NUM_STEPS
CURRICULUM_RAMP_FRACTION = 0.7  # reach full difficulty by 70% through training, refine after


class StairsCurriculum(BaseCallback):
    """Linearly ramps two difficulty axes over the first `ramp_fraction` of
    training, instead of presenting the full difficulty from step zero:
      - step height: CURRICULUM_START_HEIGHT_RANGE -> STEP_HEIGHT_RANGE
      - active step count: 1 -> NUM_STEPS (steps beyond the active count stay
        flat, see envs/terrain.py's `active_steps`)
    See mujoco_rl/README.md for why: a fixed full-difficulty staircase from
    step zero let the policy learn to just fall/freeze at the first step
    rather than climb it."""

    def __init__(self, total_timesteps, end_height_range=STEP_HEIGHT_RANGE,
                 start_height_range=CURRICULUM_START_HEIGHT_RANGE,
                 end_active_steps=NUM_STEPS, start_active_steps=CURRICULUM_START_ACTIVE_STEPS,
                 ramp_fraction=CURRICULUM_RAMP_FRACTION, update_freq=20_000, verbose=0):
        super().__init__(verbose)
        self.total_timesteps = total_timesteps
        self.start_height_range = start_height_range
        self.end_height_range = end_height_range
        self.start_active_steps = start_active_steps
        self.end_active_steps = end_active_steps
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
        active_steps = round(self.start_active_steps + t * (self.end_active_steps - self.start_active_steps))
        self.training_env.env_method("set_step_height_range", low, high)
        self.training_env.env_method("set_active_steps", active_steps)
        if self.verbose:
            print(f"[curriculum] progress={t:.2f} step_height_range=({low:.4f}, {high:.4f}) active_steps={active_steps}")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-test", action="store_true", help="short run (~100k steps) to verify the training loop end-to-end")
    parser.add_argument("--timesteps", type=int, default=20_000_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--checkpoint-freq", type=int, default=200_000)
    parser.add_argument("--run-name", type=str, default="stairs")
    parser.add_argument("--warm-start", type=str, default=None, help="path to a checkpoint (e.g. the flat-ground walk policy) to initialize weights from")
    parser.add_argument("--no-curriculum", action="store_true", help="disable the step-height curriculum, use the full STEP_HEIGHT_RANGE from step zero")
    parser.add_argument("--learning-rate", type=float, default=3e-4, help="lower this (e.g. 5e-5) when continuing training from a checkpoint that already works, to fine-tune instead of risking large destabilizing updates")
    args = parser.parse_args()

    timesteps = 100_000 if args.smoke_test else args.timesteps
    n_envs = 2 if args.smoke_test else args.n_envs

    vec_env = make_vec_env(DogzillaStairsEnv, n_envs=n_envs, vec_env_cls=SubprocVecEnv)

    if args.warm_start:
        # Passing learning_rate here overrides the checkpoint's saved value --
        # SB3 loads all other hyperparameters (clip_range, gamma, ...) from the
        # checkpoint unless also overridden.
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
        name_prefix="ppo_dogzilla_stairs",
    )

    callbacks = [checkpoint_callback]
    if not args.no_curriculum:
        callbacks.append(StairsCurriculum(total_timesteps=timesteps, verbose=1))

    model.learn(total_timesteps=timesteps, callback=CallbackList(callbacks), progress_bar=True)
    model.save(str(run_dir / "final_model"))
    print(f"Saved final model to {run_dir / 'final_model'}.zip")


if __name__ == "__main__":
    main()

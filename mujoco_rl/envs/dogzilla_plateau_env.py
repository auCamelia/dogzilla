"""Gymnasium environment: DOGZILLA plateau crossing -- step UP onto a raised
platform, walk across, step DOWN back to ground level.

Same action/observation shape and shared reward machinery as
DogzillaStairsEnv (terrain-relative height, progress, foot clearance, hip
splay/joint-extremity penalties, stall-window early termination -- see that
file's docstring and mujoco_rl/README.md's "Stairs environment" section for
why each of these exists). The difference is purely the terrain shape: one
up-step + one down-step (envs/terrain.py's `build_plateau_xml`) instead of a
multi-step staircase, and a plateau-height curriculum instead of a
step-height + step-count one.
"""
import collections

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from envs.dogzilla_env import (
    FALL_HEIGHT_LIMIT,
    FALL_TILT_LIMIT,
    HIP_JOINT_INDICES,
    JOINT_EXTREME_MARGIN,
    JOINT_EXTREME_WEIGHT,
    LATENCY_STEPS_RANGE,
    LEG_FLEX_JOINT_INDICES,
    MASS_RANDOMIZATION_RANGE,
    MOTOR_STRENGTH_RANGE,
    NUM_JOINTS,
    REST_POSE_RAD,
    TARGET_HEIGHT,
    DogzillaWalkEnv,
    quat_to_euler,
)
from envs.terrain import (
    PLATEAU_GEOM_NAME,
    PLATEAU_LANDING_GEOM_NAME,
    PLATEAU_LENGTH,
    PLATEAU_START_GEOM_NAME,
    build_plateau_xml,
    randomize_plateau_height,
)

PLATEAU_HEIGHT_RANGE = (0.01, 0.04)  # "small step" up/down, same span as the stairs task
FORWARD_SPEED_RANGE = (0.1, 0.2)  # m/s, no lateral/yaw command -- cross straight-on
MAX_EPISODE_STEPS = 700
GROUND_FRICTION_RANGE = (0.5, 1.1)
STALL_WINDOW = 100  # control steps (~2s) over which forward progress is checked
STALL_MIN_PROGRESS = 0.05  # meters; less than this over STALL_WINDOW steps ends the episode

FOOT_SITE_NAMES = ["lf_foot", "rf_foot", "lh_foot", "rh_foot"]
FOOT_CLEARANCE_CAP = 0.06
FOOT_CLEARANCE_WEIGHT = 8.0
CROSSING_BONUS = 10.0  # one-time bonus for stepping up onto the plateau, and again for stepping down off it

GROUND_GEOM_NAMES = [PLATEAU_START_GEOM_NAME, PLATEAU_GEOM_NAME, PLATEAU_LANDING_GEOM_NAME]


class DogzillaPlateauEnv(DogzillaWalkEnv):
    def __init__(self, randomize=True):
        # Deliberately skip DogzillaWalkEnv.__init__ (different model/XML source);
        # duplicate only the parts of its setup that still apply unchanged.
        gym.Env.__init__(self)
        self.randomize = randomize
        xml = build_plateau_xml()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.frame_skip = max(1, round(0.02 / self.model.opt.timestep))

        self.joint_ids = np.arange(1, NUM_JOINTS + 1)
        self.joint_qpos_adr = self.model.jnt_qposadr[self.joint_ids]
        self.joint_qvel_adr = self.model.jnt_dofadr[self.joint_ids]
        self.joint_range = self.model.jnt_range[self.joint_ids].copy()

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(NUM_JOINTS,), dtype=np.float32)
        obs_dim = NUM_JOINTS + NUM_JOINTS + 3 + 3 + NUM_JOINTS
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

        self._prev_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._cmd = np.zeros(3, dtype=np.float32)
        self._step_count = 0
        self._rng = np.random.default_rng()
        self._plateau_height = PLATEAU_HEIGHT_RANGE[0]
        self._plateau_height_range = PLATEAU_HEIGHT_RANGE  # overridden per-training-run by a curriculum callback
        self._foot_site_ids = [self.model.site(name).id for name in FOOT_SITE_NAMES]

        self._nominal_body_mass = self.model.body_mass.copy()
        self._nominal_body_inertia = self.model.body_inertia.copy()
        self._nominal_gainprm = self.model.actuator_gainprm.copy()
        self._nominal_biasprm = self.model.actuator_biasprm.copy()
        self._ground_geom_ids = [self.model.geom(name).id for name in GROUND_GEOM_NAMES]
        self._nominal_ground_friction = self.model.geom_friction[self._ground_geom_ids].copy()
        self._base_body_id = self.model.body("base_link").id

        self._action_buffer = collections.deque(maxlen=LATENCY_STEPS_RANGE[1] + 1)
        self._next_push_in = 0

    def set_plateau_height_range(self, low, high):
        """Called via VecEnv.env_method by a curriculum callback during training."""
        self._plateau_height_range = (low, high)

    def _randomize_physics(self):
        mass_scale = self._rng.uniform(*MASS_RANDOMIZATION_RANGE)
        self.model.body_mass[:] = self._nominal_body_mass * mass_scale
        self.model.body_inertia[:] = self._nominal_body_inertia * mass_scale

        friction_scale = self._rng.uniform(*GROUND_FRICTION_RANGE)
        for geom_id, nominal in zip(self._ground_geom_ids, self._nominal_ground_friction):
            self.model.geom_friction[geom_id] = nominal * friction_scale

        strength_scale = self._rng.uniform(*MOTOR_STRENGTH_RANGE)
        self.model.actuator_gainprm[:] = self._nominal_gainprm * strength_scale
        self.model.actuator_biasprm[:] = self._nominal_biasprm * strength_scale

    def _expected_ground_height(self, x):
        if 0 <= x < PLATEAU_LENGTH:
            return self._plateau_height
        return 0.0

    def _sample_command(self):
        vx = self._rng.uniform(*FORWARD_SPEED_RANGE)
        return np.array([vx, 0.0, 0.0], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self._plateau_height = self._rng.uniform(*self._plateau_height_range)
        randomize_plateau_height(self.model, self._plateau_height)
        if self.randomize:
            self._randomize_physics()
        self._reset_latency_and_pushes()

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = -0.5  # start midway across the flat platform, facing the plateau
        self.data.qpos[2] = 0.15
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[self.joint_qpos_adr] = REST_POSE_RAD
        self.data.ctrl[:] = REST_POSE_RAD
        mujoco.mj_forward(self.model, self.data)

        self._prev_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._cmd = self._sample_command()
        self._step_count = 0
        self._prev_x = float(self.data.qpos[0])
        self._stall_window_x = self._prev_x
        self._stall_window_step = 0
        self._crossed_up = False  # stepped onto the plateau
        self._crossed_down = False  # stepped back down onto the landing
        return self._get_obs(), {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        self._action_buffer.append(action)
        delayed_action = self._action_buffer[0]
        target = self._scale_action(delayed_action)
        self.data.ctrl[:] = target

        if self.randomize:
            self._maybe_apply_push()
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self._base_body_id, :2] = 0

        obs = self._get_obs()
        roll, pitch, _ = quat_to_euler(self.data.qpos[3:7])
        base_x = self.data.qpos[0]
        base_z = self.data.qpos[2]
        ground_z = self._expected_ground_height(base_x)
        relative_height = base_z - ground_z
        vx, vy, vyaw = self._base_planar_velocity()

        lin_err = (vx - self._cmd[0]) ** 2 + (vy - self._cmd[1]) ** 2
        yaw_err = (vyaw - self._cmd[2]) ** 2
        # Tight sigma -- see envs/dogzilla_stairs_env.py's comment: a loose one
        # lets the policy "pass" by standing still instead of actually crossing.
        tracking_lin = np.exp(-lin_err / 0.02)
        tracking_yaw = np.exp(-yaw_err / 0.25)
        upright = np.exp(-(roll ** 2 + pitch ** 2) / 0.4)
        height = np.exp(-((relative_height - TARGET_HEIGHT) ** 2) / 0.001)
        hip_angles = obs[HIP_JOINT_INDICES]
        hip_splay_penalty = -1.0 * np.mean(np.square(hip_angles))

        leg_angles = obs[LEG_FLEX_JOINT_INDICES]
        leg_low = self.joint_range[LEG_FLEX_JOINT_INDICES, 0]
        leg_high = self.joint_range[LEG_FLEX_JOINT_INDICES, 1]
        leg_normalized = (leg_angles - leg_low) / (leg_high - leg_low)
        leg_extremity = np.maximum(leg_normalized, 1.0 - leg_normalized)
        joint_extreme_penalty = -JOINT_EXTREME_WEIGHT * float(np.mean(np.clip(leg_extremity - JOINT_EXTREME_MARGIN, 0.0, None)))

        action_rate_penalty = -0.01 * np.sum((action - self._prev_action) ** 2)
        torque_penalty = -1e-4 * np.sum(np.square(self.data.actuator_force))
        survival = 0.5

        progress = base_x - self._prev_x
        progress_reward = 50.0 * progress
        self._prev_x = base_x

        foot_clearances = []
        for site_id in self._foot_site_ids:
            foot_x = self.data.site_xpos[site_id, 0]
            foot_z = self.data.site_xpos[site_id, 2]
            clearance = np.clip(foot_z - self._expected_ground_height(foot_x), 0.0, FOOT_CLEARANCE_CAP)
            foot_clearances.append(clearance)
        foot_clearance_reward = FOOT_CLEARANCE_WEIGHT * float(np.mean(foot_clearances))

        # One-time bonuses: stepping up onto the plateau, then stepping back
        # down off it -- two sparse checkpoints alongside the dense progress term.
        crossing_bonus = 0.0
        if not self._crossed_up and base_x > 0.05:
            self._crossed_up = True
            crossing_bonus += CROSSING_BONUS
        if not self._crossed_down and base_x > PLATEAU_LENGTH + 0.05:
            self._crossed_down = True
            crossing_bonus += CROSSING_BONUS

        reward = (
            1.0 * tracking_lin
            + 0.5 * tracking_yaw
            + 0.3 * upright
            + 1.0 * height
            + progress_reward
            + foot_clearance_reward
            + crossing_bonus
            + hip_splay_penalty
            + joint_extreme_penalty
            + action_rate_penalty
            + torque_penalty
            + survival
        )

        fallen = abs(roll) > FALL_TILT_LIMIT or abs(pitch) > FALL_TILT_LIMIT or relative_height < FALL_HEIGHT_LIMIT
        self._step_count += 1

        if self._step_count - self._stall_window_step >= STALL_WINDOW:
            if base_x - self._stall_window_x < STALL_MIN_PROGRESS:
                fallen = True
            self._stall_window_x = base_x
            self._stall_window_step = self._step_count

        truncated = self._step_count >= MAX_EPISODE_STEPS

        self._prev_action = action
        return obs, float(reward), bool(fallen), bool(truncated), {}

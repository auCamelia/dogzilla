"""Gymnasium environment: DOGZILLA small-stair climbing.

Same action space and (blind, proprioception-only) observation shape as
DogzillaWalkEnv — no terrain/exteroception in the observation, matching the
real robot's lack of a depth/height sensor. Differences from DogzillaWalkEnv:
  - Fixed forward-walking command (crossing the stairs straight-on) instead of
    a randomly sampled omnidirectional velocity command.
  - The standing-height reward/fall check are computed relative to the local
    stair terrain height under the robot, not an absolute world Z (which would
    otherwise be wrong the moment the robot is standing on a step).
  - Step height is randomized every episode (see envs/terrain.py) within
    STEP_HEIGHT_RANGE; step count/depth/width are fixed at compile time.
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
    TARGET_HEIGHT,
    DogzillaWalkEnv,
    quat_to_euler,
)
from envs.terrain import (
    LANDING_GEOM_NAME,
    NUM_STEPS,
    START_GEOM_NAME,
    STEP_DEPTH,
    STEP_GEOM_NAMES,
    build_stairs_xml,
    randomize_step_height,
)

STEP_HEIGHT_RANGE = (0.01, 0.04)  # "small stairs" per the roadmap, ~1-4cm rise
FORWARD_SPEED_RANGE = (0.1, 0.2)  # m/s, no lateral/yaw command — cross the stairs straight-on
MAX_EPISODE_STEPS = 700  # enough time to cross ~2.4m of track at these speeds
GROUND_FRICTION_RANGE = (0.5, 1.1)
STALL_WINDOW = 100  # control steps (~2s) over which forward progress is checked
STALL_MIN_PROGRESS = 0.05  # meters; less than this over STALL_WINDOW steps ends the episode

FOOT_SITE_NAMES = ["lf_foot", "rf_foot", "lh_foot", "rh_foot"]
FOOT_CLEARANCE_CAP = 0.06  # meters; caps the reward so flailing legs isn't free reward
FOOT_CLEARANCE_WEIGHT = 8.0
STEP_CROSSED_BONUS = 10.0  # one-time bonus the first time base_x clears each step's far edge

GROUND_GEOM_NAMES = [START_GEOM_NAME] + STEP_GEOM_NAMES + [LANDING_GEOM_NAME]


class DogzillaStairsEnv(DogzillaWalkEnv):
    def __init__(self, randomize=True):
        # Deliberately skip DogzillaWalkEnv.__init__ (different model/XML source);
        # duplicate only the parts of its setup that still apply unchanged.
        gym.Env.__init__(self)
        self.randomize = randomize
        xml = build_stairs_xml()
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
        self._step_height = STEP_HEIGHT_RANGE[0]
        self._step_height_range = STEP_HEIGHT_RANGE  # overridden per-training-run by a curriculum callback
        self._active_steps = NUM_STEPS  # overridden per-training-run by a curriculum callback
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

    def set_step_height_range(self, low, high):
        """Called via VecEnv.env_method by a curriculum callback during training."""
        self._step_height_range = (low, high)

    def set_active_steps(self, active_steps):
        """Called via VecEnv.env_method by a curriculum callback during training."""
        self._active_steps = int(round(np.clip(active_steps, 1, NUM_STEPS)))

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
        if x < 0:
            return 0.0
        step_idx = int(x // STEP_DEPTH)
        if step_idx < NUM_STEPS:
            return min(step_idx + 1, self._active_steps) * self._step_height
        return self._active_steps * self._step_height

    def _sample_command(self):
        vx = self._rng.uniform(*FORWARD_SPEED_RANGE)
        return np.array([vx, 0.0, 0.0], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self._step_height = self._rng.uniform(*self._step_height_range)
        randomize_step_height(self.model, self._step_height, active_steps=self._active_steps)
        if self.randomize:
            self._randomize_physics()
        self._reset_latency_and_pushes()

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = -0.5  # start midway across the flat platform, facing the stairs
        self.data.qpos[2] = 0.15
        self.data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)

        self._prev_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._cmd = self._sample_command()
        self._step_count = 0
        self._prev_x = float(self.data.qpos[0])
        self._stall_window_x = self._prev_x
        self._stall_window_step = 0
        self._steps_crossed = [False] * NUM_STEPS
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
        # Tighter sigma than the flat-walk task: at cmd~0.15m/s, the old 0.25
        # sigma gave standing completely still ~0.91 reward (exp(-0.0225/0.25))
        # -- almost as good as actually walking -- which let the policy learn
        # to just freeze at the first step instead of climbing it.
        tracking_lin = np.exp(-lin_err / 0.02)
        tracking_yaw = np.exp(-yaw_err / 0.25)
        upright = np.exp(-(roll ** 2 + pitch ** 2) / 0.4)
        height = np.exp(-((relative_height - TARGET_HEIGHT) ** 2) / 0.001)
        hip_angles = obs[HIP_JOINT_INDICES]
        hip_splay_penalty = -1.0 * np.mean(np.square(hip_angles))

        # Discourage knees locking straight at their range limit (observed
        # directly in the viewer: legs looked rigid/extended instead of bent),
        # without suppressing the normal mid-range swing motion of walking.
        leg_angles = obs[LEG_FLEX_JOINT_INDICES]
        leg_low = self.joint_range[LEG_FLEX_JOINT_INDICES, 0]
        leg_high = self.joint_range[LEG_FLEX_JOINT_INDICES, 1]
        leg_normalized = (leg_angles - leg_low) / (leg_high - leg_low)
        leg_extremity = np.maximum(leg_normalized, 1.0 - leg_normalized)
        joint_extreme_penalty = -JOINT_EXTREME_WEIGHT * float(np.mean(np.clip(leg_extremity - JOINT_EXTREME_MARGIN, 0.0, None)))

        action_rate_penalty = -0.01 * np.sum((action - self._prev_action) ** 2)
        torque_penalty = -1e-4 * np.sum(np.square(self.data.actuator_force))
        survival = 0.5
        # Raw net forward displacement this step, in meters -- a direct, dense
        # "did you actually move forward" signal that tracking_lin alone can't
        # give (tracking rewards matching instantaneous speed, not covering ground).
        progress = base_x - self._prev_x
        progress_reward = 50.0 * progress
        self._prev_x = base_x

        # Foot clearance above the *local* terrain under that foot specifically
        # (not the body): ~0 for any properly planted foot regardless of which
        # step it's on, positive only while a foot is actively lifted -- a direct
        # incentive to lift legs higher, which plain velocity/progress tracking
        # doesn't teach on its own (the policy was face-planting into risers).
        foot_clearances = []
        for site_id in self._foot_site_ids:
            foot_x = self.data.site_xpos[site_id, 0]
            foot_z = self.data.site_xpos[site_id, 2]
            clearance = np.clip(foot_z - self._expected_ground_height(foot_x), 0.0, FOOT_CLEARANCE_CAP)
            foot_clearances.append(clearance)
        foot_clearance_reward = FOOT_CLEARANCE_WEIGHT * float(np.mean(foot_clearances))

        # One-time bonus the first time the robot's base clears each step's far
        # edge -- a sparse "checkpoint" signal alongside the dense progress term.
        step_bonus = 0.0
        for k in range(NUM_STEPS):
            if not self._steps_crossed[k] and base_x > (k + 1) * STEP_DEPTH:
                self._steps_crossed[k] = True
                step_bonus += STEP_CROSSED_BONUS

        reward = (
            1.0 * tracking_lin
            + 0.5 * tracking_yaw
            + 0.3 * upright
            + 1.0 * height
            + progress_reward
            + foot_clearance_reward
            + step_bonus
            + hip_splay_penalty
            + joint_extreme_penalty
            + action_rate_penalty
            + torque_penalty
            + survival
        )

        fallen = abs(roll) > FALL_TILT_LIMIT or abs(pitch) > FALL_TILT_LIMIT or relative_height < FALL_HEIGHT_LIMIT
        self._step_count += 1

        # Standing still is "safe" (no fall penalty) but shouldn't be free: if a
        # fall ends the episode and forfeits future reward, a purely risk-averse
        # policy can learn to just freeze near the stairs instead of attempting
        # the climb. Ending the episode on a lack of real progress removes that
        # loophole the same way an actual fall would.
        if self._step_count - self._stall_window_step >= STALL_WINDOW:
            if base_x - self._stall_window_x < STALL_MIN_PROGRESS:
                fallen = True
            self._stall_window_x = base_x
            self._stall_window_step = self._step_count

        truncated = self._step_count >= MAX_EPISODE_STEPS

        self._prev_action = action
        return obs, float(reward), bool(fallen), bool(truncated), {}

"""Gymnasium environment: DOGZILLA velocity-command-tracking walking task.

Action and observation spaces are deliberately restricted to what the real
robot can actually command/sense through DOGZILLALib:
  - action:      12 joint position targets  (-> DOGZILLALib.motor() at deployment)
  - observation: 12 joint angles + 12 joint velocities (finite-differenced on
                 hardware by differencing consecutive read_motor() calls) +
                 roll/pitch/yaw (-> read_roll/pitch/yaw()) + commanded velocity
                 + previous action.
No body linear/angular velocity or acceleration is included — the real IMU
can't provide it (see README "Reinforcement Learning (MuJoCo)" section).

Domain randomization (per episode, when randomize=True): body mass/inertia,
floor friction, actuator gain ("motor strength"), action latency, and random
push perturbations — covering the main sim2real gaps (unmodeled payload,
unknown floor surface, servo-to-servo/voltage variation, serial round-trip
delay, and unmodeled disturbances).
"""
import collections
import pathlib

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

MODEL_PATH = pathlib.Path(__file__).resolve().parent.parent / "models" / "dogzilla.xml"

NUM_JOINTS = 12
CONTROL_DT = 0.02  # 50 Hz control rate
TARGET_HEIGHT = 0.127  # measured settled standing height, see scripts/view_model.py --headless
FALL_TILT_LIMIT = 0.9  # rad (~51.5 deg) — episode ends past this
FALL_HEIGHT_LIMIT = 0.04  # m
MAX_EPISODE_STEPS = 500

CMD_VX_RANGE = (-0.3, 0.3)
CMD_VY_RANGE = (-0.15, 0.15)
CMD_VYAW_RANGE = (-1.0, 1.0)

MASS_RANDOMIZATION_RANGE = (0.8, 1.2)  # scales every body's mass + inertia
FLOOR_FRICTION_RANGE = (0.5, 1.1)  # sliding friction coefficient
MOTOR_STRENGTH_RANGE = (0.7, 1.3)  # scales every actuator's kp/kv
LATENCY_STEPS_RANGE = (0, 2)  # control-step action delay (0-40ms at 50Hz)
PUSH_INTERVAL_STEPS_RANGE = (100, 300)  # control steps between pushes
PUSH_FORCE_RANGE = (0.0, 15.0)  # newtons, random horizontal direction

HIP_JOINT_INDICES = [0, 3, 6, 9]  # lf/rf/lh/rh hip, within the 12-joint arrays
LEG_FLEX_JOINT_INDICES = [1, 2, 4, 5, 7, 8, 10, 11]  # upper/lower leg joints (knees), not hips
JOINT_EXTREME_MARGIN = 0.85  # only penalize within the outer 15% of a joint's range
JOINT_EXTREME_WEIGHT = 3.0  # discourages locking legs straight at the range limit


def quat_to_euler(quat):
    """wxyz quaternion -> (roll, pitch, yaw) in radians."""
    w, x, y, z = quat
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class DogzillaWalkEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, randomize=True):
        super().__init__()
        self.randomize = randomize
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.frame_skip = max(1, round(CONTROL_DT / self.model.opt.timestep))

        self.joint_ids = np.arange(1, NUM_JOINTS + 1)  # joint 0 is the base freejoint
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

        # Nominal physical parameters, restored/rescaled at the start of every episode.
        self._nominal_body_mass = self.model.body_mass.copy()
        self._nominal_body_inertia = self.model.body_inertia.copy()
        self._nominal_geom_friction = self.model.geom_friction.copy()
        self._nominal_gainprm = self.model.actuator_gainprm.copy()
        self._nominal_biasprm = self.model.actuator_biasprm.copy()
        self._floor_geom_id = self.model.geom("floor").id
        self._base_body_id = self.model.body("base_link").id

        self._action_buffer = collections.deque(maxlen=LATENCY_STEPS_RANGE[1] + 1)
        self._next_push_in = 0

    def _randomize_physics(self):
        mass_scale = self._rng.uniform(*MASS_RANDOMIZATION_RANGE)
        self.model.body_mass[:] = self._nominal_body_mass * mass_scale
        self.model.body_inertia[:] = self._nominal_body_inertia * mass_scale

        friction_scale = self._rng.uniform(*FLOOR_FRICTION_RANGE)
        self.model.geom_friction[self._floor_geom_id] = self._nominal_geom_friction[self._floor_geom_id] * friction_scale

        strength_scale = self._rng.uniform(*MOTOR_STRENGTH_RANGE)
        self.model.actuator_gainprm[:] = self._nominal_gainprm * strength_scale
        self.model.actuator_biasprm[:] = self._nominal_biasprm * strength_scale

    def _reset_latency_and_pushes(self):
        latency_steps = int(self._rng.integers(LATENCY_STEPS_RANGE[0], LATENCY_STEPS_RANGE[1] + 1))
        # maxlen = latency_steps + 1 so that, once full, buffer[0] is exactly
        # `latency_steps` control steps older than the action just appended.
        self._action_buffer = collections.deque(
            [np.zeros(NUM_JOINTS, dtype=np.float32)] * latency_steps, maxlen=latency_steps + 1,
        )
        self._next_push_in = self._rng.integers(*PUSH_INTERVAL_STEPS_RANGE)

    def _maybe_apply_push(self):
        self._next_push_in -= 1
        if self._next_push_in > 0:
            return
        angle = self._rng.uniform(0, 2 * np.pi)
        force = self._rng.uniform(*PUSH_FORCE_RANGE)
        self.data.xfrc_applied[self._base_body_id, 0] = force * np.cos(angle)
        self.data.xfrc_applied[self._base_body_id, 1] = force * np.sin(angle)
        self._next_push_in = self._rng.integers(*PUSH_INTERVAL_STEPS_RANGE)

    def _sample_command(self):
        vx = self._rng.uniform(*CMD_VX_RANGE)
        vy = self._rng.uniform(*CMD_VY_RANGE)
        vyaw = self._rng.uniform(*CMD_VYAW_RANGE)
        return np.array([vx, vy, vyaw], dtype=np.float32)

    def _scale_action(self, action):
        low, high = self.joint_range[:, 0], self.joint_range[:, 1]
        return low + (action + 1.0) * 0.5 * (high - low)

    def _get_obs(self):
        joint_pos = self.data.qpos[self.joint_qpos_adr]
        joint_vel = self.data.qvel[self.joint_qvel_adr]
        roll, pitch, yaw = quat_to_euler(self.data.qpos[3:7])
        return np.concatenate([
            joint_pos, joint_vel,
            [roll, pitch, yaw],
            self._cmd,
            self._prev_action,
        ]).astype(np.float32)

    def _base_planar_velocity(self):
        """Base linear/yaw velocity expressed in the base's own heading frame."""
        lin_vel_world = self.data.qvel[0:3]
        ang_vel_world = self.data.qvel[3:6]
        _, _, yaw = quat_to_euler(self.data.qpos[3:7])
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        vx = cos_y * lin_vel_world[0] + sin_y * lin_vel_world[1]
        vy = -sin_y * lin_vel_world[0] + cos_y * lin_vel_world[1]
        vyaw = ang_vel_world[2]
        return vx, vy, vyaw

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        if self.randomize:
            self._randomize_physics()
        self._reset_latency_and_pushes()

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[2] = 0.15  # matches models/dogzilla.xml base_link start height
        self.data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)

        self._prev_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._cmd = self._sample_command()
        self._step_count = 0
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
        base_z = self.data.qpos[2]
        vx, vy, vyaw = self._base_planar_velocity()

        lin_err = (vx - self._cmd[0]) ** 2 + (vy - self._cmd[1]) ** 2
        yaw_err = (vyaw - self._cmd[2]) ** 2
        tracking_lin = np.exp(-lin_err / 0.25)
        tracking_yaw = np.exp(-yaw_err / 0.25)
        upright = np.exp(-(roll ** 2 + pitch ** 2) / 0.4)
        # Tight sigma (~2cm std) so crouching is actually penalized instead of
        # being a nearly-free way to trade height for easier velocity tracking.
        height = np.exp(-((base_z - TARGET_HEIGHT) ** 2) / 0.001)
        hip_angles = obs[HIP_JOINT_INDICES]
        hip_splay_penalty = -1.0 * np.mean(np.square(hip_angles))

        # Discourage knees locking straight at their range limit, without
        # suppressing the normal mid-range swing motion of walking.
        leg_angles = obs[LEG_FLEX_JOINT_INDICES]
        leg_low = self.joint_range[LEG_FLEX_JOINT_INDICES, 0]
        leg_high = self.joint_range[LEG_FLEX_JOINT_INDICES, 1]
        leg_normalized = (leg_angles - leg_low) / (leg_high - leg_low)
        leg_extremity = np.maximum(leg_normalized, 1.0 - leg_normalized)
        joint_extreme_penalty = -JOINT_EXTREME_WEIGHT * float(np.mean(np.clip(leg_extremity - JOINT_EXTREME_MARGIN, 0.0, None)))

        action_rate_penalty = -0.01 * np.sum((action - self._prev_action) ** 2)
        torque_penalty = -1e-4 * np.sum(np.square(self.data.actuator_force))
        survival = 0.5

        reward = (
            1.0 * tracking_lin
            + 0.5 * tracking_yaw
            + 0.3 * upright
            + 1.0 * height
            + hip_splay_penalty
            + joint_extreme_penalty
            + action_rate_penalty
            + torque_penalty
            + survival
        )

        fallen = abs(roll) > FALL_TILT_LIMIT or abs(pitch) > FALL_TILT_LIMIT or base_z < FALL_HEIGHT_LIMIT
        self._step_count += 1
        truncated = self._step_count >= MAX_EPISODE_STEPS

        self._prev_action = action
        return obs, float(reward), bool(fallen), bool(truncated), {}

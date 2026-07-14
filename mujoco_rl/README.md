# DOGZILLA — MuJoCo Reinforcement Learning

Dev-PC-only MuJoCo + Gymnasium + Stable-Baselines3 (PPO) pipeline for teaching DOGZILLA
locomotion skills beyond its onboard firmware: walking, small-stair climbing, and fall
recovery. See the root `README.md`'s "Reinforcement Learning (MuJoCo)" chapter for how
this fits into the overall project and why the action/observation spaces are shaped the
way they are (they mirror what `DOGZILLALib` can actually command/sense on real hardware).

## Setup

```bash
cd mujoco_rl
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

This venv is independent from the Pi's ROS 2 Docker image — training only ever runs here,
on the dev PC.

## Usage

**1 — Sanity-check the model** (always do this first after touching `models/dogzilla.xml`):

```bash
python3 scripts/view_model.py             # interactive viewer, robot drops and settles
python3 scripts/view_model.py --headless  # prints settle diagnostics only, no viewer window
```

Confirms the robot settles into a stable, upright standing pose under gravity — a
collapsed or tipped-over result means the model (masses/inertias/joint ranges/contacts)
needs fixing before any training is worth running.

**2 — Train a walking policy:**

```bash
python3 training/train_walk.py --smoke-test   # ~100k steps, a few minutes — verifies the full loop
python3 training/train_walk.py                # full run, 20M steps by default
```

Checkpoints are saved under `checkpoints/<run-name>/`. Roll out a saved checkpoint
visually by loading it in `scripts/view_model.py`-style viewer code with SB3's
`PPO.load(...)` (a dedicated rollout/eval script will be added in a follow-up phase).

## Model (`models/dogzilla.xml`)

Hand-authored MJCF, not an automatic URDF conversion. Geometry, meshes, and inertials
come from `yahboomcar_ws/src/yahboom_description/urdf/yahboom_xgo_rviz.xacro`; joint
ranges come from `DOGZILLALib.PARAM["MOTOR_LIMIT"]` (the real servo angle limits in
degrees: hip `[-73,57]`, upper leg `[-66,93]`, lower leg `[-31,31]`) since the xacro's own
joint limits are unfilled `±π` placeholders. Joint/actuator naming mirrors DOGZILLALib's
motor id convention (x1 = hip, x2 = upper leg, x3 = lower leg) so a trained action maps
directly onto `DOGZILLALib.motor(motor_id, angle_deg)` at deployment time.

Collision uses the visual meshes directly (MuJoCo's implicit convex hull) rather than
hand-placed primitives — a reasonable first-pass approximation; revisit if foot contact
behavior looks wrong once gaits start forming.

Servo PD gains (`kp`/`kv` on the `<position>` actuators) are a first-pass guess, since
`DOGZILLALib` doesn't expose real servo gain/torque constants — tune by feel in
`view_model.py`, or via real-robot system identification later.

## Environment (`envs/dogzilla_env.py`)

`DogzillaWalkEnv` — velocity-command tracking on flat ground.

- **Action** (12,): joint position targets, normalized to `[-1, 1]` and scaled to each
  joint's real hardware range.
- **Observation** (42,): 12 joint angles + 12 joint velocities + roll/pitch/yaw + commanded
  velocity (vx, vy, vyaw) + previous action. No body linear/angular velocity or
  acceleration — the real IMU can't provide it, so the sim doesn't get it either.
- **Reward**: exponential velocity-tracking (linear + yaw) + upright-orientation term +
  standing-height term + small action-rate/torque penalties + a survival bonus.
- **Episode end**: falls (roll/pitch past ~51°, or base height collapses) end the
  episode; otherwise truncates at 500 steps (10s at the 50 Hz control rate).

## Roadmap status

| Phase | Goal | Status |
|---|---|---|
| 1 | MJCF model + base Gymnasium env + PPO walking smoke-test | Done |
| 2 | Full walking policy + domain randomization | Not started |
| 3 | Small-stair climbing (procedural terrain) | Not started |
| 4 | Fall recovery | Not started |
| 5 | Sim-to-real deployment (new ROS 2 node, calls `DOGZILLALib.motor()` directly) | Not started |
| 6 | Nav2 integration | Open question, not decided |

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

Checkpoints are saved under `checkpoints/<run-name>/` (gitignored — trained models aren't
committed). `python3 training/train_walk.py --timesteps 5000000 --n-envs 8` is the budget
that actually produced the current `checkpoints/walk/final_model.zip`.

**3 — Watch a trained policy:**

```bash
python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip                       # interactive viewer
python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip --record out.mp4 --no-randomize  # headless, no display needed
```

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

## How the learning actually works

This section walks through the mechanics behind `training/train_walk.py`, concretely, so
"it learns to walk" isn't a black box.

**1 — The environment is just a physics simulator, not a learner.** `DogzillaWalkEnv`
(`envs/dogzilla_env.py`) does nothing clever: `reset()` drops the robot back to a standing
pose and samples a random target velocity; `step(action)` applies 12 joint targets,
advances MuJoCo's physics by one control tick (20ms = 10 physics substeps of 2ms), and
returns the new observation plus a reward number. That's it — all the "intelligence"
lives outside this class, in the PPO agent.

**2 — The agent is a small neural network.** `PPO("MlpPolicy", ...)` creates a network
(SB3's default: two 64-unit hidden layers) that maps the 42-number observation to two
things: a probability distribution over the 12 actions (the *policy*), and a single number
estimating "how good is this situation" (the *value function*). At the start its weights
are random, so it acts randomly — which is exactly why the very first training steps saw
the robot collapse after ~28 steps every time.

**3 — Data collection happens in parallel.** `SubprocVecEnv` runs 8 independent copies of
the environment as separate OS processes (one per CPU core). Every iteration, PPO lets all
8 run for `n_steps=1024` steps each — 8192 `(observation, action, reward, next
observation)` transitions collected in one batch. This is the expensive part (MuJoCo
physics on the CPU), not the neural network itself — which is why the GPU sits nearly
idle during training (a small MLP policy gives it almost nothing to do) while all 8 CPU
cores stay busy running physics.

**4 — Advantage estimation (GAE).** For every transition in the batch, PPO asks: was the
outcome better or worse than the value function predicted? It combines rewards several
steps into the future (`gamma=0.99` discount) with a smoothing term (`gae_lambda=0.95`)
into a single "advantage" number. Positive advantage = "do more of this in this
situation"; negative = "do less."

**5 — The clipped policy update.** With the 8192-transition batch, PPO runs `n_epochs=10`
passes over it in `batch_size=256` mini-batches, each time nudging the network to increase
the probability of high-advantage actions and decrease low-advantage ones (ordinary
gradient descent). The "Proximal" in PPO comes from `clip_range=0.2`: it caps how far the
policy is allowed to move in one update, so one unlucky batch of data can't undo
everything learned so far.

**6 — Repeat thousands of times.** One iteration (collect 8192 steps + 10 update epochs)
takes a handful of seconds; a 5M-step run is ~610 iterations, about 1h15 on this machine.
No single iteration teaches much — the gait *emerges* over thousands of iterations purely
because certain joint-angle sequences consistently score higher reward than others. Nobody
wrote "swing the left-front leg forward, then the right-hind leg" anywhere in this repo.

**7 — The reward is the entire specification, for better and worse.** The network only
ever tries to maximize the number returned by `step()`'s reward calculation — it has no
other notion of "correct." This is exactly what produced the crab-crawl bug documented
below: the reward at the time genuinely was maximized by that gait, so PPO found it. PPO
optimizes precisely what you measure, not what you meant — which is why watching a
rendered rollout (`scripts/rollout_policy.py --record`) matters as much as the reward
curves, if not more.

## Environment (`envs/dogzilla_env.py`)

`DogzillaWalkEnv` — velocity-command tracking on flat ground.

- **Action** (12,): joint position targets, normalized to `[-1, 1]` and scaled to each
  joint's real hardware range.
- **Observation** (42,): 12 joint angles + 12 joint velocities + roll/pitch/yaw + commanded
  velocity (vx, vy, vyaw) + previous action. No body linear/angular velocity or
  acceleration — the real IMU can't provide it, so the sim doesn't get it either.
- **Reward**: exponential velocity-tracking (linear + yaw) + upright-orientation term +
  standing-height term (tight sigma, weight 1.0) + hip-splay penalty (mean squared hip
  joint angle) + small action-rate/torque penalties + a survival bonus.
- **Episode end**: falls (roll/pitch past ~51°, or base height collapses) end the
  episode; otherwise truncates at 500 steps (10s at the 50 Hz control rate).

> **Lesson learned (Phase 2):** the first reward version (height weight 0.2, loose sigma,
> no hip term) let PPO find a locally-optimal but degenerate "crab-crawl" gait — crouched
> low with hips splayed sideways, which tracked commanded velocity fine but isn't a real
> walk (roll/pitch stayed near zero since the body itself stayed level, so the
> upright term didn't catch it). Caught by actually watching a recorded rollout, not by
> reward curves — `ep_rew_mean` looked completely healthy the whole time. Fixed by
> tightening the height sigma + raising its weight so crouching is punished, and adding
> `hip_splay_penalty` on the 4 hip joints. **Always eyeball a rendered rollout
> (`scripts/rollout_policy.py --record`) before trusting reward curves alone.**

### Domain randomization (Phase 2, `randomize=True` by default)

Resampled every episode reset, to close the main sim2real gaps:

| Parameter | Range | Models |
|---|---|---|
| Body mass + inertia | 0.8x - 1.2x nominal (all bodies, same factor) | unmodeled payload/wiring/battery mass |
| Floor friction | 0.5x - 1.1x nominal | unknown real-world surface (carpet/tile/wood) |
| Actuator gain (kp/kv) | 0.7x - 1.3x nominal | servo-to-servo variation, battery voltage sag |
| Action latency | 0-2 control steps (0-40ms @ 50Hz) | real serial round-trip delay |
| Random pushes | 0-15N horizontal, every 100-300 steps | unmodeled disturbances/collisions |

Pass `DogzillaWalkEnv(randomize=False)` for a clean nominal-physics rollout (e.g. to
separate "policy is bad" from "policy just hasn't adapted to this randomized sample") —
this is also what `scripts/rollout_policy.py --no-randomize` uses under the hood.

## Roadmap status

| Phase | Goal | Status |
|---|---|---|
| 1 | MJCF model + base Gymnasium env + PPO walking smoke-test | Done |
| 2 | Full walking policy + domain randomization | Done |
| 3 | Small-stair climbing (procedural terrain) | Not started |
| 4 | Fall recovery | Not started |
| 5 | Sim-to-real deployment (new ROS 2 node, calls `DOGZILLALib.motor()` directly) | Not started |
| 6 | Nav2 integration | Open question, not decided |

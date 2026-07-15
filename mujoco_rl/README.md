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

**3 — Train a stair-climbing policy** (warm-starting from the walk policy converges far
faster than random init — see "How the learning actually works" below):

```bash
python3 training/train_stairs.py --timesteps 6000000 --n-envs 8 \
    --run-name stairs --warm-start checkpoints/walk/final_model.zip
```

**4 — Watch a trained policy:**

```bash
python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip                       # interactive viewer, real time
python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip --record out.mp4 --no-randomize  # headless, no display needed
python3 scripts/rollout_policy.py checkpoints/stairs/final_model.zip --env stairs --speed 0.5  # slow-motion, stairs task
```

See "Command reference" below for every flag on every script.

## Model (`models/dogzilla.xml`)

Hand-authored MJCF, not an automatic URDF conversion. Geometry, meshes, and inertials
come from `yahboomcar_ws/src/yahboom_description/urdf/yahboom_xgo_rviz.xacro`; joint
ranges come from `DOGZILLALib.PARAM["MOTOR_LIMIT"]` (the real servo angle *spans* in
degrees: hip `[-73,57]`, upper leg `[-66,93]`, lower leg `[-31,31]`) since the xacro's own
joint limits are unfilled `±π` placeholders. Joint/actuator naming mirrors DOGZILLALib's
motor id convention (x1 = hip, x2 = upper leg, x3 = lower leg) so a trained action maps
directly onto `DOGZILLALib.motor(motor_id, angle_deg)` at deployment time.

Collision uses the visual meshes directly (MuJoCo's implicit convex hull) rather than
hand-placed primitives — a reasonable first-pass approximation; revisit if foot contact
behavior looks wrong once gaits start forming.

> **Lesson learned (joint-range offset, 2026-07-15 — invalidates all prior checkpoints):**
> `MOTOR_LIMIT`'s spans are centered on the real robot's *power-on/calibration* pose (its
> factory `Software calibration.pdf`: thigh-body and calf-thigh both folded to 90°), **not**
> on this mesh's own zero pose (a fully straight leg — an unrelated CAD-export artifact).
> The model originally applied `MOTOR_LIMIT` centered on straight-leg-zero, which was
> simply wrong — it let the sim explore a window of motion the real robot's firmware can't
> reach, and started every episode from an unreachable, unnatural straight-leg pose (this
> is almost certainly what "the gait looks like a paraplegic's" and "the rear legs are
> rigid" feedback, mid-Phase-2/3, was really pointing at, on top of the reward-shaping
> issues documented elsewhere in this file).
>
> Found by comparing photos of the real robot's rest pose against rendered sim poses,
> then confirmed by hand with `scripts/tune_pose.py` (a full interactive MuJoCo GUI with
> per-joint control sliders, gravity disabled so the robot floats instead of falling while
> you pose it) against the physical unit. Measured real rest pose, in this mesh's
> straight-leg-zero convention: hip=0°, upper leg=+35.7°/-35.7° (left/right), lower
> leg=-85.0°/+85.0° (left/right) — see `envs/dogzilla_env.py`'s `REST_POSE_RAD`. The
> `upper`/`lower` default classes were split into `upper_left`/`upper_right`/`lower_left`/
> `lower_right`, each `MOTOR_LIMIT`'s span re-centered on the matching side's measured
> angle instead of on 0 (hip needed no such correction — measured at 0° on both sides,
> matching the mesh's own zero). Every environment now starts episodes from `REST_POSE_RAD`
> instead of all-zero joint angles. **This changes the action space's meaning entirely —
> every checkpoint trained before this fix is stale and cannot be warm-started from.**

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

**8 — Warm-starting: reusing a network instead of starting from random weights.**
`training/train_stairs.py --warm-start checkpoints/walk/final_model.zip` calls
`PPO.load(path, env=vec_env)`, which restores the *exact* trained weights (policy +
value function) from the flat-walk run instead of the random initialization normally
used. Training then continues on top of that, against the new stairs environment/reward.
Concretely this matters a lot early on: from scratch, `ep_rew_mean` after 100k steps was
~157; warm-started from the walk policy, it was already ~700-1000 within the first few
thousand steps, because "how to stand up and move forward" transfers directly. The
trade-off: the network also inherits whatever the walk policy's gait was tuned around, so
a flaw baked into that base policy (see the stiff-rear-leg lesson below) rides along into
every task warm-started from it.

**9 — Continuing training safely: lowering the learning rate.** Naively continuing to
train an already-decent checkpoint at the same `learning_rate=3e-4` used for its original
training can make things *worse*, not better — this actually happened during Phase 3
(a continuation run's `approx_kl` spiked to ~0.09-0.10 and `clip_fraction` to ~0.6,
both signs of large, destabilizing policy updates, and the resulting checkpoint could no
longer cross even one step, versus its starting point). `train_stairs.py --learning-rate`
overrides the checkpoint's saved learning rate at load time
(`PPO.load(path, env=vec_env, learning_rate=...)`); dropping it from `3e-4` to `5e-5` for
a continuation run keeps updates small enough to fine-tune rather than overwrite what
already worked.

## Environment (`envs/dogzilla_env.py`)

`DogzillaWalkEnv` — velocity-command tracking on flat ground.

- **Action** (12,): joint position targets, normalized to `[-1, 1]` and scaled to each
  joint's real hardware range.
- **Observation** (42,): 12 joint angles + 12 joint velocities + roll/pitch/yaw + commanded
  velocity (vx, vy, vyaw) + previous action. No body linear/angular velocity or
  acceleration — the real IMU can't provide it, so the sim doesn't get it either.
- **Reward**: exponential velocity-tracking (linear + yaw) + upright-orientation term +
  standing-height term (tight sigma, weight 1.0) + hip-splay penalty (mean squared hip
  joint angle) + joint-extremity penalty (discourages knees locking straight at their
  range limit) + small action-rate/torque penalties + a survival bonus.
- **Episode end**: falls (roll/pitch past ~51°, or base height collapses) end the
  episode; otherwise truncates at 500 steps (10s at the 50 Hz control rate).

> **Lesson learned (Phase 2, crab-crawl):** the first reward version (height weight 0.2,
> loose sigma, no hip term) let PPO find a locally-optimal but degenerate "crab-crawl"
> gait — crouched low with hips splayed sideways, which tracked commanded velocity fine
> but isn't a real walk (roll/pitch stayed near zero since the body itself stayed level,
> so the upright term didn't catch it). Caught by actually watching a recorded rollout,
> not by reward curves — `ep_rew_mean` looked completely healthy the whole time. Fixed by
> tightening the height sigma + raising its weight so crouching is punished, and adding
> `hip_splay_penalty` on the 4 hip joints. **Always eyeball a rendered rollout
> (`scripts/rollout_policy.py --record`) before trusting reward curves alone.**

> **Lesson learned (stiff legs):** watching a *trained* walk policy in the real-time
> interactive viewer (`scripts/rollout_policy.py --speed 0.5`, no `--record`) — not the
> distant, low-res still frames used to sanity-check Phase 2 the first time around —
> revealed the rear legs locking straight/rigid instead of staying bent. Nothing in the
> reward discouraged a joint from sitting at the extreme of its range; a straight leg
> satisfies height/upright/tracking just as well as a bent one, so PPO had no reason to
> avoid it. Fixed with `joint_extreme_penalty`: it computes each knee-like joint's position
> normalized into `[0, 1]` across its range, and only penalizes the outer 15%
> (`JOINT_EXTREME_MARGIN = 0.85`) — normal mid-range swing motion during walking is
> untouched, only sitting at (or very near) the hard limit is discouraged. This is why
> the walking policy was retrained from scratch rather than patched: it's the base
> checkpoint every other task warm-starts from (see "Warm-starting" above), so a defect
> here silently rides along into stairs, fall recovery, everything downstream.

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

## Stairs environment (`envs/dogzilla_stairs_env.py`, `envs/terrain.py`, `training/train_stairs.py`)

`DogzillaStairsEnv` subclasses `DogzillaWalkEnv` (reusing action scaling, observation
construction, domain randomization, action-latency, and push-perturbation code as-is) but
replaces the flat floor with a procedurally generated staircase, and swaps in a
forward-only command and a terrain-aware reward. Status: partially working (the policy
reliably approaches the stairs and, at the easy end of the difficulty range, has crossed
the first step) but not yet a reliable full climb — this section documents the mechanism
and the string of failure modes found and fixed along the way, since that history is the
most useful thing to know before touching this code further.

**Terrain generation (`envs/terrain.py`).** Rather than hand-authoring a second MJCF file
(risking the robot body definition drifting out of sync between two files),
`build_stairs_xml()` reads `models/dogzilla.xml` as plain text and textually replaces its
single `<geom name="floor".../>` line with a start platform + `NUM_STEPS=5` box-step geoms
+ a landing platform. MuJoCo can't add or remove geoms without recompiling the whole
model, so the step *count* and *depth/width* are fixed forever once compiled — but step
*height* is changed every episode by directly overwriting the already-compiled model's
`geom_size`/`geom_pos` arrays (`randomize_step_height()`), the same trick used for mass/
friction/motor-gain domain randomization in the base env, and just as cheap (no
recompilation).

**Curriculum (`training/train_stairs.py`'s `StairsCurriculum` callback).** Presenting the
full difficulty (5 steps, up to 4cm rise) from the first training step turned out to
teach nothing (see the lessons below) — so `StairsCurriculum` linearly ramps two axes
over the first `ramp_fraction=0.7` of training:
- **Step height**: `(0.0, 0.005)` → `STEP_HEIGHT_RANGE = (0.01, 0.04)` meters.
- **Active step count**: `1` → `NUM_STEPS=5`. Steps beyond the "active" count are
  flattened to the height of the last active step (`envs/terrain.py`'s `_step_rise()`),
  so the track is always geometrically continuous — the robot just doesn't have to climb
  any of the flattened ones yet.

Every `update_freq=20_000` steps it calls `training_env.env_method("set_step_height_range",
low, high)` and `set_active_steps(n)` on every parallel env — the standard way to reach
into a `SubprocVecEnv`'s remote environments from a callback.

**Reward** — velocity tracking + upright + hip-splay + joint-extremity (shared with the
walk task) plus stairs-specific terms:
- **Terrain-relative height**, not absolute world Z: `_expected_ground_height(x)`
  computes the analytic staircase profile so the height reward means "how far above
  *whatever's directly underneath* the robot is standing" — using absolute Z would
  wrongly penalize the robot for simply being higher up after successfully climbing.
- **Progress** (`progress_reward`): raw net x-displacement this step, in meters, times a
  weight of 50 — a dense, direct "did you actually move forward" signal, deliberately
  separate from velocity tracking (see the "freezing" lesson below).
- **Foot clearance** (`foot_clearance_reward`): for each of the 4 foot sites, clearance =
  `foot_z - _expected_ground_height(foot_x)`, clipped to `[0, 0.06m]`. A planted foot has
  ~0 clearance regardless of which step it's on; only an actively lifted foot scores, which
  directly rewards lifting a leg higher to clear a riser without needing explicit
  swing/stance phase detection.
- **Per-step checkpoint bonus** (`step_bonus`): a one-time `+10` the first time the robot's
  base clears each step's far edge — a sparse signal layered on top of the dense progress
  reward.

**Episode ends early on lack of progress, not just on falling.** `STALL_WINDOW=100`
control steps: if net x-displacement over that window is under `STALL_MIN_PROGRESS=0.05m`,
the episode ends exactly like a fall would. See the "freezing" lesson below for why this
existed.

### Lessons learned building the stairs task (chronological)

> **1 — Loose velocity-tracking sigma → policy stands still and calls it "tracking."**
> The walk task's `tracking_lin` sigma (`0.25`) gives standing *completely* still ~0.91
> reward when the command is only ~0.15 m/s (`exp(-0.15²/0.25)`) — nearly as good as
> actually walking. On flat ground this never mattered enough to notice; in front of a
> stair riser (a genuinely hard, fall-risking obstacle) it was enough incentive to just
> not try. Fixed by tightening `tracking_lin`'s sigma to `0.02` for the stairs task
> specifically, and adding the separate `progress_reward` (distance-based, not
> velocity-based) described above.

> **2 — Even with progress reward, PPO learned to freeze.** Falling ends the episode and
> forfeits all future reward; standing still safely near the stairs (collecting survival +
> upright + height every step, ~2.5/step) for a full 700-step episode (~1750 total) beat
> the expected value of attempting a risky climb and probably falling partway through.
> This is a standard "risk-averse freezing" failure mode, not specific to this codebase.
> Fixed with the stall-window early termination above: freezing now ends the episode just
> as fast as falling does, removing the "safe to do nothing" option entirely.

> **3 — Naive "train longer" made things *worse*, not better.** Continuing training
> from a checkpoint that had (rarely) crossed step one, at the same `learning_rate=3e-4`
> used originally, drove `approx_kl` up to ~0.09-0.10 and `clip_fraction` to ~0.6 (both
> signs of large, destabilizing updates) and the result regressed to crossing zero steps
> across a full evaluation batch. See "Continuing training safely" above — the fix was
> `--learning-rate 5e-5` for continuation runs, not more steps at the original rate.

> **4 — Stiff/straight legs.** Same root cause and same fix as the walk-policy lesson
> above (`joint_extreme_penalty`); the stairs task warm-starts from the walk policy, so it
> inherited the defect until the base policy was retrained.

## Command reference

All commands assume `cd mujoco_rl && source .venv/bin/activate` first.

### `scripts/view_model.py` — sanity-check the MJCF model

| Flag | Default | Meaning |
|---|---|---|
| `--headless` | off | skip the interactive viewer window, just print settle diagnostics (base height/orientation after 3s) and exit |

```bash
python3 scripts/view_model.py             # opens a window, robot drops from 0.15m and settles
python3 scripts/view_model.py --headless  # for scripts/CI — no window, just prints numbers
```

### `scripts/tune_pose.py` — find/verify a joint pose against the real robot

Opens the *full* interactive MuJoCo GUI (not the minimal passive viewer), with gravity
disabled so the robot floats in place instead of falling/tipping over while you drag each
joint's control slider (in the "Control" panel) and compare directly against the physical
robot. No flags — everything is done live in the GUI. Angles print in radians and degrees
when you close the window. This is how `envs/dogzilla_env.py`'s `REST_POSE_RAD` was found.

```bash
python3 scripts/tune_pose.py
```

### `training/train_walk.py` and `training/train_stairs.py` — PPO training

| Flag | Default | Meaning |
|---|---|---|
| `--smoke-test` | off | forces `--timesteps 100000 --n-envs 2` — verifies the full env/PPO/checkpoint loop in a few minutes, not a real training budget |
| `--timesteps N` | `20000000` | total environment steps to train for |
| `--n-envs N` | `8` | parallel `SubprocVecEnv` copies (match to CPU core count) |
| `--checkpoint-freq N` | `200000` | environment steps between saved checkpoints |
| `--run-name NAME` | `walk` / `stairs` | checkpoints go to `checkpoints/<run-name>/` |
| `--warm-start PATH` | none | initialize weights from an existing `.zip` checkpoint instead of random init (e.g. bootstrap stairs from the walk policy) |

`train_stairs.py` additionally has:

| Flag | Default | Meaning |
|---|---|---|
| `--no-curriculum` | off | use the full step-height range and all 5 steps from step zero, skipping the height/step-count ramp-up (see "Stairs environment" above for why this alone doesn't work well) |
| `--learning-rate X` | `3e-4` | overrides the checkpoint's saved learning rate when `--warm-start` is set; lower it (e.g. `5e-5`) when continuing training from a checkpoint that already works, to avoid destabilizing it (see "Lessons learned" above) |

```bash
python3 training/train_walk.py --smoke-test
python3 training/train_walk.py --timesteps 5000000 --n-envs 8 --run-name walk

python3 training/train_stairs.py --smoke-test
python3 training/train_stairs.py --timesteps 6000000 --n-envs 8 --run-name stairs \
    --warm-start checkpoints/walk/final_model.zip
# continuing an already-decent stairs checkpoint safely:
python3 training/train_stairs.py --timesteps 6000000 --run-name stairs_v2 \
    --warm-start checkpoints/stairs/final_model.zip --no-curriculum --learning-rate 5e-5
```

### `scripts/rollout_policy.py` — watch a trained checkpoint

| Flag | Default | Meaning |
|---|---|---|
| `checkpoint` (positional) | required | path to a saved SB3 `.zip` |
| `--env {walk,stairs}` | `walk` | which task env the checkpoint was trained on |
| `--no-randomize` | off | disable domain randomization for a clean, repeatable nominal-physics rollout |
| `--episodes N` | `10` | number of episodes to play |
| `--seed N` | `0` | first episode's seed (episode *i* uses `seed+i`) — reuse a seed to replay the exact same command/step-height/etc. |
| `--record PATH.mp4` | none | render headless to this video file instead of opening the interactive viewer (no display needed) |
| `--speed X` | `1.0` | interactive-viewer-only: playback speed multiplier (`0.5` = slow motion, `2.0` = 2x) — the viewer paces itself to real time by default |

```bash
python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip
python3 scripts/rollout_policy.py checkpoints/walk/final_model.zip --speed 0.4
python3 scripts/rollout_policy.py checkpoints/stairs/final_model.zip --env stairs --no-randomize --episodes 5
python3 scripts/rollout_policy.py checkpoints/stairs/final_model.zip --env stairs --record stairs.mp4 --no-randomize
```

Close the viewer window manually (click the X) when done watching, rather than waiting
for the script to exit on its own — the passive viewer's shutdown has a known segfault-on-close
quirk on some driver/GLFW combinations after the window is closed; it happens strictly
during cleanup, after all episodes have already finished and printed their results.

## Roadmap status

| Phase | Goal | Status |
|---|---|---|
| 1 | MJCF model + base Gymnasium env + PPO walking smoke-test | Done |
| 2 | Full walking policy + domain randomization | Needs retraining — the joint-range/rest-pose fix (see "Model" above) changed the action space; prior checkpoint is stale |
| 3 | Small-stair climbing (procedural terrain) | Needs retraining, same reason as Phase 2 — prior progress (occasional single-step crossing) was against the old, incorrect joint ranges |
| 4 | Fall recovery | Not started |
| 5 | Sim-to-real deployment (new ROS 2 node, calls `DOGZILLALib.motor()` directly) | Not started |
| 6 | Nav2 integration | Open question, not decided |

"""Procedural stair terrain, built by injecting box-step geoms into the existing
flat-ground model (models/dogzilla.xml) rather than hand-authoring a second MJCF —
the robot body definition (meshes, inertials, joints, actuators) stays a single
source of truth.

Step count/depth/width are fixed at model-compile time (MuJoCo can't add/remove
geoms without recompiling); step *height* is randomized per episode by directly
mutating the compiled model's geom_size/geom_pos arrays, the same technique used
for domain randomization in envs/dogzilla_env.py — no recompilation needed.
"""
import pathlib

MODEL_PATH = pathlib.Path(__file__).resolve().parent.parent / "models" / "dogzilla.xml"
MESHDIR = pathlib.Path(__file__).resolve().parent.parent / "models" / "meshes"

NUM_STEPS = 5
STEP_DEPTH = 0.08  # m, along the walking (+x) direction
STEP_WIDTH = 0.6  # m, wide enough that lateral drift doesn't matter for a first pass
START_PLATFORM_DEPTH = 1.0  # flat ground before the stairs, to reach a steady gait first
LANDING_DEPTH = 1.0  # flat ground after the last step
NOMINAL_STEP_HEIGHT = 0.02  # placeholder baked into the initial XML; overwritten every reset

STEP_GEOM_NAMES = [f"stair_step_{k}" for k in range(NUM_STEPS)]
LANDING_GEOM_NAME = "stairs_landing"
START_GEOM_NAME = "stairs_start"

FLOOR_GEOM_LINE = '<geom name="floor" type="plane" size="5 5 0.1" material="floor" friction="0.9 0.02 0.01"/>'


def _step_rise(step_height, k, active_steps):
    """Rise (meters) of step k. Steps beyond `active_steps` stay flat at the
    height of the last active step -- a step-count curriculum (fewer active
    steps = fewer risers to solve at once) without changing the track length."""
    return min(k + 1, active_steps) * step_height


def _step_geom_dims(step_height, k, active_steps=NUM_STEPS):
    """(half_x, half_y, half_z, pos_x, pos_z) for step k at a given rise height."""
    half_x = STEP_DEPTH / 2
    half_y = STEP_WIDTH / 2
    half_z = _step_rise(step_height, k, active_steps) / 2
    pos_x = k * STEP_DEPTH + half_x
    pos_z = half_z
    return half_x, half_y, half_z, pos_x, pos_z


def _landing_geom_dims(step_height, active_steps=NUM_STEPS):
    """Solid slab from ground level up to the top of the last (active) step
    (same z=0-to-top convention as the step boxes), extending further in +x."""
    half_x = LANDING_DEPTH / 2
    half_y = STEP_WIDTH / 2
    half_z = _step_rise(step_height, NUM_STEPS - 1, active_steps) / 2
    pos_x = NUM_STEPS * STEP_DEPTH + half_x
    pos_z = half_z
    return half_x, half_y, half_z, pos_x, pos_z


def build_stairs_xml():
    """Flat-ground dogzilla.xml with its floor plane swapped for a start platform
    + a fixed-topology staircase (placeholder height, mutated per episode) + landing."""
    xml = MODEL_PATH.read_text()
    xml = xml.replace('meshdir="meshes"', f'meshdir="{MESHDIR}"')

    geoms = [
        f'<geom name="{START_GEOM_NAME}" type="box" '
        f'size="{START_PLATFORM_DEPTH / 2} {STEP_WIDTH / 2} 0.05" '
        f'pos="{-START_PLATFORM_DEPTH / 2} 0 -0.05" material="floor" friction="0.9 0.02 0.01"/>'
    ]
    for k in range(NUM_STEPS):
        half_x, half_y, half_z, pos_x, pos_z = _step_geom_dims(NOMINAL_STEP_HEIGHT, k)
        geoms.append(
            f'<geom name="{STEP_GEOM_NAMES[k]}" type="box" size="{half_x} {half_y} {half_z}" '
            f'pos="{pos_x} 0 {pos_z}" material="floor" friction="0.9 0.02 0.01"/>'
        )
    half_x, half_y, half_z, pos_x, pos_z = _landing_geom_dims(NOMINAL_STEP_HEIGHT)
    geoms.append(
        f'<geom name="{LANDING_GEOM_NAME}" type="box" size="{half_x} {half_y} {half_z}" '
        f'pos="{pos_x} 0 {pos_z}" material="floor" friction="0.9 0.02 0.01"/>'
    )

    xml = xml.replace(FLOOR_GEOM_LINE, "\n    ".join(geoms))
    return xml


def randomize_step_height(model, step_height, active_steps=NUM_STEPS):
    """Mutate the already-compiled model's stair geoms to realize `step_height`
    (meters) and `active_steps` (int, 1-NUM_STEPS) without recompiling. Call
    once per episode reset."""
    for k in range(NUM_STEPS):
        geom_id = model.geom(STEP_GEOM_NAMES[k]).id
        half_x, half_y, half_z, pos_x, pos_z = _step_geom_dims(step_height, k, active_steps)
        model.geom_size[geom_id] = [half_x, half_y, half_z]
        model.geom_pos[geom_id] = [pos_x, 0, pos_z]

    landing_id = model.geom(LANDING_GEOM_NAME).id
    half_x, half_y, half_z, pos_x, pos_z = _landing_geom_dims(step_height, active_steps)
    model.geom_size[landing_id] = [half_x, half_y, half_z]
    model.geom_pos[landing_id] = [pos_x, 0, pos_z]

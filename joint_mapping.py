"""
Simulation <-> real SO-101 joint mapping.

Simulation (MuJoCo Menagerie robotstudio_so101, derived from TheRobotStudio so101_new_calib):
  - joint angles in RADIANS, 0 = middle of each joint's range
  - gripper: hinge angle, -0.1745 rad (closed) .. 1.7453 rad (fully open)

Real arms (LeRobot SOFollower, use_degrees=True, your existing calibration):
  - body joints in DEGREES: (raw - middle_of_calibrated_range) * 360 / 4095  -> 0 = middle of range
  - gripper: 0..100 across its calibrated range (on your arms 100 = open, 0 = closed)
  - limits: +/- half of each joint's calibrated range (read from the calibration at connect time)

So for body joints:  real_deg = DIRECTION * degrees(sim_rad) + OFFSET_DEG
DIRECTION is +1 for every joint: shoulder_lift was -1 (from the webcam mapping) but real-arm
testing showed it moving opposite to the sim on both arms.
Verify with "Verify mapping" mode (V) before enabling motion: move a real arm by hand
(torque off) and the sim arm must move the same way. If a joint is mirrored, flip it here.
"""
import math

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
BODY_JOINTS = JOINTS[:5]
SIDES = ("Left", "Right")
PREFIX = {"Left": "left_", "Right": "right_"}
LABEL = {"shoulder_pan": "1 Shoulder pan", "shoulder_lift": "2 Shoulder lift",
         "elbow_flex": "3 Elbow flex", "wrist_flex": "4 Wrist flex",
         "wrist_roll": "5 Wrist roll", "gripper": "6 Gripper"}

# ---- EDIT HERE if Verify mode shows a mirrored joint (+1.0 <-> -1.0) ----
DIRECTION = {
    "Left":  {"shoulder_pan": 1.0, "shoulder_lift": 1.0, "elbow_flex": 1.0,
              "wrist_flex": 1.0, "wrist_roll": 1.0},
    "Right": {"shoulder_pan": 1.0, "shoulder_lift": 1.0, "elbow_flex": 1.0,
              "wrist_flex": 1.0, "wrist_roll": 1.0},
}
# Software offset (degrees) added on the real side, if a joint's zero doesn't line up. Default 0.
OFFSET_DEG = {s: {j: 0.0 for j in BODY_JOINTS} for s in SIDES}
# Set True for an arm whose real gripper is 0 = open, 100 = closed.
GRIPPER_INVERT = {"Left": False, "Right": False}

SIM_GRIPPER_RANGE = (-0.174533, 1.745329)  # rad, closed .. open (so101.xml)
REAL_GRIPPER_RANGE = (0.0, 100.0)
LIMIT_MARGIN_DEG = 3.0       # stay this far inside the calibrated limits
GRIPPER_MARGIN = 1.0         # % inside 0..100


def _clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def sim_to_real(side, q):
    """q: {joint: sim radians} -> {joint: real units (deg, gripper 0..100)}"""
    out = {}
    for j in BODY_JOINTS:
        out[j] = DIRECTION[side][j] * math.degrees(q[j]) + OFFSET_DEG[side][j]
    lo, hi = SIM_GRIPPER_RANGE
    frac = _clip((q["gripper"] - lo) / (hi - lo), 0.0, 1.0)
    if GRIPPER_INVERT[side]:
        frac = 1.0 - frac
    rlo, rhi = REAL_GRIPPER_RANGE
    out["gripper"] = rlo + frac * (rhi - rlo)
    return out


def real_to_sim(side, r):
    """Inverse of sim_to_real: {joint: real units} -> {joint: sim radians}"""
    q = {}
    for j in BODY_JOINTS:
        q[j] = math.radians((r[j] - OFFSET_DEG[side][j]) / DIRECTION[side][j])
    rlo, rhi = REAL_GRIPPER_RANGE
    frac = _clip((r["gripper"] - rlo) / (rhi - rlo), 0.0, 1.0)
    if GRIPPER_INVERT[side]:
        frac = 1.0 - frac
    lo, hi = SIM_GRIPPER_RANGE
    q["gripper"] = lo + frac * (hi - lo)
    return q


def real_limits_from_calibration(calibration, resolution=4096):
    """{joint: (lo, hi)} in real units from a LeRobot calibration dict, with safety margins."""
    lim = {}
    for j in BODY_JOINTS:
        c = calibration[j]
        half = (c.range_max - c.range_min) / 2 * 360.0 / (resolution - 1)
        lim[j] = (-half + LIMIT_MARGIN_DEG, half - LIMIT_MARGIN_DEG)
    lim["gripper"] = (REAL_GRIPPER_RANGE[0] + GRIPPER_MARGIN, REAL_GRIPPER_RANGE[1] - GRIPPER_MARGIN)
    return lim


def clamp_real(r, limits):
    return {j: _clip(v, *limits[j]) for j, v in r.items()}

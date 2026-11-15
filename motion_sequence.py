"""
Teach-and-replay motion sequences (e.g. folding a t-shirt) for both simulated arms.

A sequence is a list of waypoints. Each waypoint is a full pose of BOTH arms (sim radians, the
same values as JointCommands.q) plus a hold time. Saved as JSON, so you can hand-edit holds or
delete a bad waypoint.

Playback interpolates from the current pose through every waypoint with a smoothstep profile
(zero velocity at each waypoint, no jerks). Each segment's duration comes from its biggest joint
change at the play speed, so big moves take longer. The real arms follow the sim as usual
(robot_sync still rate-limits and clamps them).
"""
import json
import math
import os

from joint_mapping import JOINTS, SIDES, SIM_GRIPPER_RANGE

DEFAULT_HOLD_S = 0.4          # pause at each waypoint (lets the gripper close/open)
MIN_SEGMENT_S = 0.5
GRIPPER_SPEED_PCT_S = 80.0    # gripper opening speed during playback (% of full range per s)


def _copy_pose(q):
    return {s: {j: float(q[s][j]) for j in JOINTS} for s in SIDES}


class MotionSequence:
    def __init__(self, path):
        self.path = path
        self.waypoints = []   # [{"Left": {joint: rad}, "Right": {...}, "hold": s}]
        if os.path.exists(path):
            with open(path) as f:
                self.waypoints = json.load(f)["waypoints"]

    def save(self):
        with open(self.path, "w") as f:
            json.dump({"waypoints": self.waypoints}, f, indent=1)

    def add(self, q, hold=DEFAULT_HOLD_S):
        wp = _copy_pose(q)
        wp["hold"] = hold
        self.waypoints.append(wp)
        self.save()
        return len(self.waypoints)

    def undo(self):
        if self.waypoints:
            self.waypoints.pop()
            self.save()
        return len(self.waypoints)


def _segment_duration(a, b, speed_deg_s):
    lo, hi = SIM_GRIPPER_RANGE
    t = MIN_SEGMENT_S
    for s in SIDES:
        for j in JOINTS:
            d = abs(b[s][j] - a[s][j])
            if j == "gripper":
                t = max(t, 100.0 * d / (hi - lo) / GRIPPER_SPEED_PCT_S)
            else:
                t = max(t, math.degrees(d) / speed_deg_s)
    return t


class Player:
    """Plays waypoints starting from `start` (the current pose). sample(t) -> pose or None when done."""

    def __init__(self, start, waypoints, speed_deg_s):
        self.segments = []   # (from, to, move_s, hold_s)
        prev = _copy_pose(start)
        for wp in waypoints:
            to = _copy_pose(wp)
            self.segments.append((prev, to, _segment_duration(prev, to, speed_deg_s),
                                  float(wp.get("hold", DEFAULT_HOLD_S))))
            prev = to
        self.total = sum(m + h for _, _, m, h in self.segments)
        self.index = 0

    def sample(self, t):
        for i, (a, b, move, hold) in enumerate(self.segments):
            if t < move:
                self.index = i
                x = t / move
                x = x * x * (3.0 - 2.0 * x)   # smoothstep
                return {s: {j: a[s][j] + x * (b[s][j] - a[s][j]) for j in JOINTS} for s in SIDES}
            t -= move
            if t < hold:
                self.index = i
                return b
            t -= hold
        return None

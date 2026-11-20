"""
Render the demo video offscreen (no windows, no hardware).

Uses the same scene (Sim), joint limits, and teach-and-replay player (Player) as sim_teleop.py, so
what you see is what the app does. The overlay shows the real-arm targets that
joint_mapping.sim_to_real would send to the motors (degrees, gripper 0..100).

  MUJOCO_GL=egl python tools/record_demo.py                 # -> docs/media/demo.mp4
  python tools/record_demo.py --out my.mp4 --sequence other.json
"""
import argparse
import math
import os
import shutil
import subprocess
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import joint_mapping as jm  # noqa: E402
from joint_mapping import JOINTS, LABEL, SIDES  # noqa: E402
from motion_sequence import MotionSequence, Player  # noqa: E402
from simulation_controller import NEUTRAL, JointCommands  # noqa: E402
from sim_teleop import Sim  # noqa: E402

W, H, FPS = 1280, 720, 30
FONT = cv2.FONT_HERSHEY_SIMPLEX


def scene_args(a):
    return argparse.Namespace(no_cloth=False, cloth_n=a.cloth_n, cloth_spacing=0.025, cloth_x=0.26,
                              spacing=0.30)


class Recorder:
    def __init__(self, a):
        self.sim = Sim(scene_args(a))
        self.sim.m.vis.global_.offwidth, self.sim.m.vis.global_.offheight = W, H
        self.cmd = JointCommands(self.sim.ctrl_limits)
        self.sim.reset(self.cmd)
        self.r = mujoco.Renderer(self.sim.m, H, W)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = [0.2, 0.0, 0.1]
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 1.0, 0.0, -35.0
        self.frames = []
        self.n_frames = 0
        self.writer = cv2.VideoWriter(a.raw, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    # -- one video frame: command -> physics (real-time) -> render -> overlay
    def frame(self, title, subtitle=""):
        self.sim.write_ctrl(self.cmd)
        for s in SIDES:
            lo, hi = jm.SIM_GRIPPER_RANGE
            self.sim.update_grasp(s, (self.cmd.q[s]["gripper"] - lo) / (hi - lo))
        self.sim.step_to(self.sim.d.time + 1.0 / FPS)
        self.r.update_scene(self.sim.d, self.cam)
        img = cv2.cvtColor(self.r.render(), cv2.COLOR_RGB2BGR)
        self._overlay(img, title, subtitle)
        self.writer.write(img)
        self.n_frames += 1

    def _overlay(self, img, title, subtitle):
        band = img.copy()
        cv2.rectangle(band, (0, 0), (W, 78), (20, 20, 20), -1)
        cv2.rectangle(band, (0, H - 150), (W, H), (20, 20, 20), -1)
        cv2.addWeighted(band, 0.65, img, 0.35, 0, img)
        cv2.putText(img, title, (24, 38), FONT, 0.95, (255, 255, 255), 2, cv2.LINE_AA)
        if subtitle:
            cv2.putText(img, subtitle, (24, 66), FONT, 0.58, (200, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"t = {self.n_frames / FPS:5.1f} s", (W - 150, 38), FONT, 0.6, (180, 180, 180), 1,
                    cv2.LINE_AA)
        # real-arm targets (what robot_sync would send)
        y0 = H - 122
        cv2.putText(img, "Real-arm targets  (sim rad -> joint_mapping.sim_to_real -> deg, gripper %)",
                    (24, y0), FONT, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
        colw = (W - 48 - 110) // len(JOINTS)
        for k, j in enumerate(JOINTS):
            cv2.putText(img, LABEL[j][2:], (134 + k * colw, y0 + 26), FONT, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        for r_i, s in enumerate(SIDES):
            real = jm.sim_to_real(s, self.sim.qpos(s))
            y = y0 + 56 + r_i * 30
            cv2.putText(img, s, (24, y), FONT, 0.6, (120, 200, 255), 2, cv2.LINE_AA)
            for k, j in enumerate(JOINTS):
                unit = "%" if j == "gripper" else " deg"
                cv2.putText(img, f"{real[j]:+7.1f}{unit}", (134 + k * colw, y), FONT, 0.58, (255, 255, 255), 1,
                            cv2.LINE_AA)

    def hold(self, seconds, title, subtitle=""):
        for _ in range(int(seconds * FPS)):
            self.frame(title, subtitle)

    def move_to(self, target, seconds, title, subtitle="", orbit=0.0):
        start = {s: dict(self.cmd.q[s]) for s in SIDES}
        n = max(1, int(seconds * FPS))
        az0 = self.cam.azimuth
        for i in range(1, n + 1):
            x = i / n
            x = x * x * (3 - 2 * x)
            for s in SIDES:
                self.cmd.set_arm(s, {j: start[s][j] + x * (target[s][j] - start[s][j]) for j in JOINTS})
            self.cam.azimuth = az0 + orbit * x
            self.frame(title, subtitle)

    def close(self):
        self.writer.release()


def pose(**kw):
    """Both arms; kw: joint=(left, right) in degrees (gripper in %)."""
    from simulation_controller import pct_to_grip
    q = {s: dict(NEUTRAL) for s in SIDES}
    for j, (l, r) in kw.items():
        for s, v in (("Left", l), ("Right", r)):
            q[s][j] = pct_to_grip(v) if j == "gripper" else math.radians(v)
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "media", "demo.mp4"))
    ap.add_argument("--sequence", default=os.path.join(ROOT, "fold_tshirt.json"))
    ap.add_argument("--play-speed", type=float, default=30.0)
    ap.add_argument("--cloth-n", type=int, default=14)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    a.raw = a.out + ".raw.mp4"

    rec = Recorder(a)
    neutral = {s: dict(NEUTRAL) for s in SIDES}

    # 1. overview: orbit around the scene at neutral
    rec.hold(1.0, "SO-101 bimanual sim-to-real teleop", "MuJoCo is the master; the real arms copy its joints")
    rec.move_to(neutral, 5.0, "SO-101 bimanual sim-to-real teleop",
                "Two SO-101 arms (MuJoCo Menagerie model), 30 cm apart, deformable cloth on the table",
                orbit=-60)
    rec.move_to(neutral, 3.0, "SO-101 bimanual sim-to-real teleop", "", orbit=60)

    # 2. joint tour, BOTH arms with mirror (pan / roll mirrored), like the control panel's 'Both' mode
    tour = [
        ("shoulder_pan", 35, True), ("shoulder_lift", -35, False), ("elbow_flex", -40, False),
        ("wrist_flex", 45, False), ("wrist_roll", 70, True),
    ]
    for j, deg, mirrored in tour:
        title = f"Joint tour: {LABEL[j]}"
        sub = "Control panel 'Both' mode" + (" - mirrored on the right arm" if mirrored else "")
        p = pose(**{j: (deg, -deg if mirrored else deg)})
        rec.move_to(p, 1.2, title, sub)
        rec.move_to(neutral, 1.0, title, sub)
    for pct in (100, 0, 30):
        rec.move_to(pose(gripper=(pct, pct)), 0.8, "Joint tour: 6 Gripper", "0 % = closed, 100 % = open")

    # 3. teach & replay: play the recorded sequence exactly like the FOLD T-SHIRT button
    seq = MotionSequence(a.sequence)
    if seq.waypoints:
        rec.move_to(pose(shoulder_lift=(-20, -20), elbow_flex=(30, 30)), 1.5,
                    "Teach & replay", f"Loaded {len(seq.waypoints)} waypoints from {os.path.basename(a.sequence)}")
        player = Player({s: rec.sim.qpos(s) for s in SIDES}, seq.waypoints, a.play_speed)
        t = 0.0
        while True:
            p = player.sample(t)
            if p is None:
                break
            for s in SIDES:
                rec.cmd.set_arm(s, p[s])
            rec.frame("Teach & replay: FOLD T-SHIRT (F)",
                      f"Waypoint {player.index + 1}/{len(seq.waypoints)}  -  smoothstep, "
                      f"{a.play_speed:.0f} deg/s on the fastest joint")
            t += 1.0 / FPS
        rec.hold(1.5, "Teach & replay: sequence finished", "Real arms (when following) are capped at 60 deg/s")

    rec.move_to(neutral, 2.0, "SO-101 bimanual sim-to-real teleop", "python sim_teleop.py  -  see README.md")
    rec.hold(1.0, "SO-101 bimanual sim-to-real teleop", "python sim_teleop.py  -  see README.md")
    rec.close()

    if shutil.which("ffmpeg"):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", a.raw, "-c:v", "libx264", "-pix_fmt",
                        "yuv420p", "-crf", "26", "-preset", "slow", "-movflags", "+faststart", a.out], check=True)
        os.remove(a.raw)
    else:
        os.replace(a.raw, a.out)
    print(f"wrote {a.out} ({rec.n_frames / FPS:.1f} s)")


if __name__ == "__main__":
    main()

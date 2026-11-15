"""
SO-101 bimanual: the MuJoCo simulation is the master, the physical arms follow.

  python sim_teleop.py                      # simulation only (default, safe)
  python sim_teleop.py --mock-robots        # test the whole real-arm workflow without hardware

Two windows open: the 3D MuJoCo view and the control panel. Nothing physical moves until you
press K (connect), check the mapping with V, and enable following with ENTER + confirmation.

No webcam / MediaPipe is used anywhere in this workflow.
"""
import argparse
import os
import time
import tkinter as tk
from tkinter import messagebox

import mujoco
import mujoco.viewer
import numpy as np

import joint_mapping as jm
from joint_mapping import JOINTS, PREFIX, SIDES
from motion_sequence import MotionSequence, Player
from real_robot_controller import MockArm, RealArm
from robot_sync import CONNECTING, DISCONNECTED, FAULT, FOLLOWING, HOLDING, VERIFY, SyncManager
from simulation_controller import ControlPanel, JointCommands

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_XML = os.path.join(HERE, "robotstudio_so101", "so101.xml")
GLFW_KEY_ESCAPE = 256

SCENE = """
<mujoco model="so101_bimanual">
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"
          iterations="20" ls_iterations="20"/>
  <asset>
    <texture name="wood" type="2d" builtin="flat" rgb1="0.72 0.55 0.35" width="64" height="64"/>
    <material name="wood" texture="wood"/>
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <light pos="-1 0 1.5" dir="1 0 -1"/>
    <geom name="table" type="plane" size="1 1 0.05" material="wood"/>
    {cloth}
  </worldbody>
</mujoco>
"""
CLOTH = """
    <flexcomp name="cloth" type="grid" count="{n} {n} 1" spacing="{s} {s} {s}"
              pos="{x} 0 0.01" radius="0.004" dim="2" mass="0.1" rgba="0.2 0.4 0.85 1">
      <edge equality="true" damping="0.002"/>
      <contact selfcollide="none" condim="3" friction="1 0.005 0.0001"/>
    </flexcomp>"""
GRAB_RADIUS, GRAB_MAX_POINTS = 0.03, 3
GRAB_CLOSE_BELOW, GRAB_OPEN_ABOVE = 0.12, 0.20  # gripper opening fraction


def build_model(args):
    cloth = "" if args.no_cloth else CLOTH.format(n=args.cloth_n, s=args.cloth_spacing, x=args.cloth_x)
    world = mujoco.MjSpec.from_string(SCENE.format(cloth=cloth))
    for side, y in (("left_", args.spacing / 2), ("right_", -args.spacing / 2)):
        robot = mujoco.MjSpec.from_file(ROBOT_XML)
        world.worldbody.add_frame(pos=[0, y, 0]).attach_body(robot.worldbody.first_body(), side, "")
    n_pts = 0 if args.no_cloth else args.cloth_n ** 2
    for side in ("left_", "right_"):
        for i in range(n_pts):
            world.add_equality(type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_BODY,
                               name1=f"{side}gripper", name2=f"cloth_{i}", active=False,
                               name=f"{side}grab_{i}")
    return world.compile(), n_pts


class Sim:
    def __init__(self, args):
        self.m, n_pts = build_model(args)
        self.d = mujoco.MjData(self.m)
        self.act = {s: {j: self.m.actuator(PREFIX[s] + j).id for j in JOINTS} for s in SIDES}
        self.qadr = {s: {j: self.m.joint(PREFIX[s] + j).qposadr[0] for j in JOINTS} for s in SIDES}
        self.ctrl_limits = {s: {j: tuple(self.m.actuator_ctrlrange[self.act[s][j]]) for j in JOINTS}
                            for s in SIDES}
        self.arm_bodies = {s: {b for b in range(self.m.nbody)
                               if self.m.body(b).name.startswith(PREFIX[s])} for s in SIDES}
        self.cloth = np.array([self.m.body(f"cloth_{i}").id for i in range(n_pts)], dtype=int)
        self.grab_eq = {s: np.array([self.m.equality(f"{PREFIX[s]}grab_{i}").id for i in range(n_pts)],
                                    dtype=int) for s in SIDES}
        self.holding = {s: False for s in SIDES}
        self.last_ctrl = None

    def reset(self, commands):
        mujoco.mj_resetData(self.m, self.d)
        for s in SIDES:
            self.set_qpos(s, commands.q[s])
            self.holding[s] = False
        self.write_ctrl(commands)
        mujoco.mj_forward(self.m, self.d)

    def set_qpos(self, side, q):
        for j in JOINTS:
            self.d.qpos[self.qadr[side][j]] = q[j]
            self.d.qvel[self.m.joint(PREFIX[side] + j).dofadr[0]] = 0.0

    def qpos(self, side):
        return {j: float(self.d.qpos[self.qadr[side][j]]) for j in JOINTS}

    def write_ctrl(self, commands):
        for s in SIDES:
            for j in JOINTS:
                self.d.ctrl[self.act[s][j]] = commands.q[s][j]
        self.last_ctrl = self.d.ctrl.copy()

    def adopt_viewer_sliders(self, commands):
        """If the MuJoCo viewer's own Control sliders changed ctrl, take those values."""
        if self.last_ctrl is None:
            return
        for s in SIDES:
            for j in JOINTS:
                a = self.act[s][j]
                if abs(self.d.ctrl[a] - self.last_ctrl[a]) > 1e-6:
                    commands.set(s, j, float(self.d.ctrl[a]))

    def update_grasp(self, side, open_frac):
        if not len(self.cloth):
            return
        if not self.holding[side] and open_frac < GRAB_CLOSE_BELOW:
            p = PREFIX[side]
            tip = self.d.site(p + "gripperframe").xpos
            dist = np.linalg.norm(self.d.xpos[self.cloth] - tip, axis=1)
            near = [i for i in np.argsort(dist)[:GRAB_MAX_POINTS] if dist[i] < GRAB_RADIUS]
            if near:
                g = self.m.body(p + "gripper").id
                rot = self.d.xmat[g].reshape(3, 3)
                for i in near:
                    e = self.grab_eq[side][i]
                    self.m.eq_data[e][0:3] = rot.T @ (self.d.xpos[self.cloth[i]] - self.d.xpos[g])
                    self.m.eq_data[e][3:6] = 0.0
                    self.d.eq_active[e] = 1
                self.holding[side] = True
        elif self.holding[side] and open_frac > GRAB_OPEN_ABOVE:
            self.d.eq_active[self.grab_eq[side]] = 0
            self.holding[side] = False

    def step_to(self, t_target):
        t_target = min(t_target, self.d.time + 0.1)
        while self.d.time < t_target:
            mujoco.mj_step(self.m, self.d)


class App:
    def __init__(self, args):
        self.args = args
        self.sim = Sim(args)
        self.cmd = JointCommands(self.sim.ctrl_limits)
        self.sim.reset(self.cmd)

        Arm = MockArm if args.mock_robots else RealArm
        ports = {"Left": (args.left_port, args.left_id), "Right": (args.right_port, args.right_id)}
        self.sync = SyncManager({s: Arm(s, *ports[s]) for s in SIDES}, hz=args.hz)

        seq_path = args.sequence if os.path.isabs(args.sequence) else os.path.join(HERE, args.sequence)
        self.seq = MotionSequence(seq_path)
        self.player = None
        self.play_t = 0.0

        self.paused = False
        self.estop_from_viewer = False
        self.note = "Simulation only. Real arms are not connected."
        self.filtered = {s: self.sim.qpos(s) for s in SIDES}

        self.root = tk.Tk()
        self.panel = ControlPanel(self.root, self.cmd, self)
        self.viewer = mujoco.viewer.launch_passive(self.sim.m, self.sim.d, key_callback=self._viewer_key)
        self.viewer.cam.lookat[:] = [0.2, 0.0, 0.1]
        self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = 1.0, 0.0, -35.0
        self.t_last = time.time()
        self.sim_clock0 = time.time()
        self.last_panel = 0.0
        self.running = True
        self.viewer_closed = False

    # ---------- actions (panel buttons / keys) ----------
    def _viewer_key(self, key):  # runs in the viewer thread: only set a flag
        if key == GLFW_KEY_ESCAPE:
            self.estop_from_viewer = True

    def connect_real(self):
        if all(a.state in (HOLDING, FOLLOWING, VERIFY, CONNECTING) for a in self.sync.arms.values()):
            self.note = "Arms already connected."
            return
        self.sync.connect()
        self.note = (f"Connecting Left={self.args.left_port}, Right={self.args.right_port}. "
                     "Arms will hold their current pose - nothing moves.")

    def toggle_verify(self):
        states = [a.state for a in self.sync.arms.values()]
        if VERIFY in states:
            self.sync.set_verify(False)
            self.note = "Verify mode off: torque back on, holding the current pose."
            return
        if not any(s in (HOLDING, FOLLOWING) for s in states):
            self.panel.info("Verify mapping", "Connect the arms first (K).")
            return
        if not messagebox.askokcancel(
                "Verify mapping",
                "Torque will be switched OFF on both arms so you can move them by hand.\n"
                "SUPPORT THE ARMS - they can drop under gravity.\n\n"
                "Move one real joint at a time: the matching SIM arm and joint must move the same way.\n"
                "If one is mirrored, change its DIRECTION in joint_mapping.py.", parent=self.root):
            return
        self.sync.set_verify(True)
        self.note = "VERIFY: move the real arms by hand; the sim mirrors them. Press V again to finish."

    def toggle_real(self):
        if self.sync.sync_enabled:
            self.sync.disable_sync()
            self.note = "Real following OFF. Arms hold their pose."
            return
        ready = [s for s in SIDES if self.sync.state(s) == HOLDING]
        if not ready:
            self.panel.info("Enable real robots",
                            "No arm is ready. Connect with K (and leave Verify mode with V) first.")
            return
        diff = 0.0
        for s in ready:
            meas = self.sync.arms[s].measured
            target = jm.sim_to_real(s, self.sim.qpos(s))
            if meas:
                diff = max(diff, max(abs(target[j] - meas[j]) for j in jm.BODY_JOINTS))
        answer = self.panel.confirm_enable(diff)
        if answer is None:
            return
        if answer:  # snap the simulation to the real pose: zero motion on enable
            for s in ready:
                q = jm.real_to_sim(s, self.sync.arms[s].measured)
                self.cmd.set_arm(s, q)
                self.sim.set_qpos(s, self.cmd.q[s])
                self.filtered[s] = dict(self.cmd.q[s])
            mujoco.mj_forward(self.sim.m, self.sim.d)
        self._push_targets(force=True)
        self.paused = False
        self.sync.enable_sync()
        not_ready = [s for s in SIDES if s not in ready]
        self.note = "REAL ROBOTS FOLLOWING." + (f" Not following: {', '.join(not_ready)}." if not_ready else "")

    # ---------- teach & replay (e.g. t-shirt fold) ----------
    def record_waypoint(self):
        if self.player:
            self.panel.info("Record waypoint", "Stop playback first (O).")
            return
        n = self.seq.add(self.cmd.q)  # in Verify mode cmd.q mirrors the real arms
        self.note = f"Waypoint {n} saved to {os.path.basename(self.seq.path)}."

    def undo_waypoint(self):
        if self.player:
            return
        n = self.seq.undo()
        self.note = f"Last waypoint removed. {n} left."

    def toggle_play(self):
        if self.player:
            self.stop_playback("Playback stopped.")
            return
        if not self.seq.waypoints:
            self.panel.info("Play sequence", "No waypoints yet. Pose the arms and press P to record some.")
            return
        if any(a.state == VERIFY for a in self.sync.arms.values()):
            self.panel.info("Play sequence", "Leave Verify mode first (V).")
            return
        self.cmd.neutral_active = False
        self.player = Player({s: self.sim.qpos(s) for s in SIDES}, self.seq.waypoints, self.args.play_speed)
        self.play_t = 0.0
        real = "real arms follow" if self.sync.sync_enabled else "SIM ONLY preview (real following is off)"
        self.note = (f"PLAYING {len(self.seq.waypoints)} waypoints, ~{self.player.total:.0f} s - {real}. "
                     "O stops, SPACE pauses, ESC e-stop.")

    def fold_tshirt(self):
        """One button: play the taught fold; with connected arms, enable following first."""
        if self.player:
            self.stop_playback("Fold stopped. Arms hold their pose.")
            return
        if not self.seq.waypoints:
            self.panel.info("Fold t-shirt",
                            "The fold hasn't been taught yet.\n\n"
                            "1. K (connect), V (verify: torque off - support the arms)\n"
                            "2. Move the real arms through the fold by hand, press P at every key pose\n"
                            "3. V again, then press this button.")
            return
        if any(a.state == VERIFY for a in self.sync.arms.values()):
            self.panel.info("Fold t-shirt", "Leave Verify mode first (V).")
            return
        if not self.sync.sync_enabled and any(self.sync.state(s) == HOLDING for s in SIDES):
            self.toggle_real()            # its own confirmation dialog; cancel = no fold
            if not self.sync.sync_enabled:
                return
        self.toggle_play()

    def stop_playback(self, note=None):
        if self.player:
            self.player = None
            if note:
                self.note = note

    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused != self.sync.pause_event.is_set():
            self.sync.toggle_pause()
        self.note = "PAUSED: simulation frozen, real arms hold." if self.paused else "Resumed."

    def emergency_stop(self):
        self.sync.emergency_stop()
        self.stop_playback()
        self.cmd.neutral_active = False
        self.note = "EMERGENCY STOP: no commands sent. Arms hold. ENTER re-enables (with confirmation)."

    def neutral(self):
        self.stop_playback()
        self.cmd.start_neutral()
        self.note = "Moving the simulation toward the neutral pose (middle of all ranges)."

    def reset_sim(self):
        self.stop_playback()
        self.cmd.reset()
        self.sim.reset(self.cmd)
        self.filtered = {s: self.sim.qpos(s) for s in SIDES}
        self.sim_clock0 = time.time()
        self.note = "Simulation reset." + (" Real arms move there slowly." if self.sync.sync_enabled else "")

    def quit(self):
        if self.sync.any_connected() and not messagebox.askokcancel(
                "Quit", "Quitting disconnects the arms and switches torque OFF (LeRobot default).\n"
                        "Support the arms or put them in a rest pose first. Quit?", parent=self.root):
            return
        self.running = False
        self.sync.shutdown()
        try:
            self.viewer.close()
        except Exception:
            pass
        self.root.destroy()

    # ---------- loop ----------
    def _push_targets(self, force=False):
        a = 0.35  # low-pass on sim joint positions -> smooth real motion, no physics jitter
        for s in SIDES:
            q = self.sim.qpos(s)
            for j in JOINTS:
                self.filtered[s][j] = q[j] if force else self.filtered[s][j] + a * (q[j] - self.filtered[s][j])
        self.sync.set_targets({s: jm.sim_to_real(s, self.filtered[s]) for s in SIDES})

    def _mode(self):
        states = [a.state for a in self.sync.arms.values()]
        if self.sync.estop_event.is_set() and self.sync.any_connected():
            return "ESTOP"
        if VERIFY in states:
            return "VERIFY"
        if self.sync.sync_enabled and FOLLOWING in states:
            return "PAUSED" if self.paused else "REAL"
        if self.paused:
            return "PAUSED"
        if self.sync.any_connected():
            return "CONNECTED"
        return "SIM"

    def _status(self):
        arms = {}
        for s in SIDES:
            a = self.sync.arms[s]
            port = a.arm.port
            text, color = {
                DISCONNECTED: (f"not connected ({port})", "#555"),
                CONNECTING: (f"connecting {port}...", "#b7791f"),
                HOLDING: (f"connected {port} - holding pose", "#2f855a"),
                VERIFY: (f"{port} TORQUE OFF - verify by hand", "#dd6b20"),
                FOLLOWING: (f"{port} FOLLOWING the simulation", "#c53030"),
                FAULT: (f"FAULT {port}: {a.error}", "#c53030"),
            }[a.state]
            arms[s] = {"text": text[:90], "color": color, "measured": a.measured,
                       "sent": a.last_sent if a.state == FOLLOWING else None}
        return {"mode": self._mode(), "note": self.note, "arms": arms}

    def tick(self):
        if not self.running:
            return
        if not self.viewer.is_running():
            if not self.viewer_closed:
                self.viewer_closed = True
                if self.sync.sync_enabled:
                    self.emergency_stop()
                self.note = "3D window closed: following stopped. Close this panel to quit."
            self.panel.refresh(self._status())
            self.root.after(200, self.tick)
            return
        if self.estop_from_viewer:
            self.estop_from_viewer = False
            self.emergency_stop()

        now = time.time()
        dt, self.t_last = now - self.t_last, now
        sim = self.sim

        # verify mode: the sim mirrors the real arms (read-only)
        for s in SIDES:
            a = self.sync.arms[s]
            if a.state == VERIFY and a.measured:
                self.cmd.set_arm(s, jm.real_to_sim(s, a.measured))

        sim.adopt_viewer_sliders(self.cmd)

        # mouse: Ctrl+drag an arm link in the 3D view -> that arm becomes compliant and keeps the pose
        pert = self.viewer.perturb
        if pert.active and pert.select > 0:
            for s in SIDES:
                if pert.select in sim.arm_bodies[s]:
                    self.cmd.set_arm(s, sim.qpos(s))

        if self.player and not self.paused:
            self.play_t += dt
            pose = self.player.sample(self.play_t)
            if pose is None:
                self.stop_playback(f"Sequence finished ({len(self.seq.waypoints)} waypoints).")
            else:
                for s in SIDES:
                    self.cmd.set_arm(s, pose[s])

        self.cmd.tick(dt)
        with self.viewer.lock():
            sim.write_ctrl(self.cmd)
            for s in SIDES:
                lo, hi = jm.SIM_GRIPPER_RANGE
                sim.update_grasp(s, (self.cmd.q[s]["gripper"] - lo) / (hi - lo))
            if self.paused:
                self.sim_clock0 = now - sim.d.time
            else:
                sim.step_to(now - self.sim_clock0)
                if sim.d.time < now - self.sim_clock0 - 0.5:
                    self.sim_clock0 = now - sim.d.time
        self.viewer.sync()

        self._push_targets()

        if now - self.last_panel > 0.1:
            self.last_panel = now
            self.panel.refresh(self._status())
        self.root.after(5, self.tick)

    def run(self):
        self.root.after(50, self.tick)
        self.root.mainloop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left-port", default="COM7")
    ap.add_argument("--right-port", default="COM9")
    ap.add_argument("--left-id", default="left_follower")
    ap.add_argument("--right-id", default="right_follower")
    ap.add_argument("--mock-robots", action="store_true", help="fake arms, to test without hardware")
    ap.add_argument("--hz", type=float, default=30.0, help="real-arm command rate")
    ap.add_argument("--spacing", type=float, default=0.30, help="distance between arm bases (m)")
    ap.add_argument("--sequence", default="fold_tshirt.json",
                    help="waypoint file for record (P) / play (O); relative = next to this script")
    ap.add_argument("--play-speed", type=float, default=30.0,
                    help="playback speed of the fastest joint (deg/s); real arms are capped at 60")
    ap.add_argument("--no-cloth", action="store_true")
    ap.add_argument("--cloth-n", type=int, default=14)
    ap.add_argument("--cloth-spacing", type=float, default=0.025)
    ap.add_argument("--cloth-x", type=float, default=0.26)
    App(ap.parse_args()).run()


if __name__ == "__main__":
    main()

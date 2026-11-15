"""
Joint commands for the two simulated arms + the Tkinter control panel (keyboard, sliders, buttons).

The JointCommands object is the single source of truth for what the simulated arms are asked to do
(MuJoCo position-actuator targets, radians). Keyboard, sliders, the MuJoCo viewer's own Control
sliders and mouse dragging in the 3D view all edit it.
"""
import math
import tkinter as tk
from tkinter import messagebox

from joint_mapping import BODY_JOINTS, JOINTS, LABEL, SIDES, SIM_GRIPPER_RANGE

NEUTRAL = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0,
           "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 0.3}  # middle of every range
STEP_DEG, STEP_DEG_FAST = 1.0, 5.0          # UP/DOWN, SHIFT+UP/DOWN
NEUTRAL_SPEED_DEG_S = 45.0                  # 'C' moves the sim toward neutral at this speed
MIRRORED = ("shoulder_pan", "wrist_roll")   # joints mirrored when driving both arms together


def grip_to_pct(rad):
    lo, hi = SIM_GRIPPER_RANGE
    return 100.0 * (rad - lo) / (hi - lo)


def pct_to_grip(pct):
    lo, hi = SIM_GRIPPER_RANGE
    return lo + pct / 100.0 * (hi - lo)


class JointCommands:
    def __init__(self, ctrl_limits):
        self.limits = ctrl_limits  # {side: {joint: (lo, hi)}} radians
        self.q = {s: dict(NEUTRAL) for s in SIDES}
        self.neutral_active = False
        self.mirror = True

    def clamp(self, side, j, v):
        lo, hi = self.limits[side][j]
        return min(hi, max(lo, v))

    def set(self, side, j, v):
        self.neutral_active = False
        self.q[side][j] = self.clamp(side, j, v)

    def set_arm(self, side, values):
        for j, v in values.items():
            self.q[side][j] = self.clamp(side, j, v)

    def nudge(self, side_sel, j, delta_rad):
        self.neutral_active = False
        sides = SIDES if side_sel == "Both" else (side_sel,)
        for s in sides:
            d = delta_rad
            if side_sel == "Both" and self.mirror and j in MIRRORED and s == "Right":
                d = -d
            self.q[s][j] = self.clamp(s, j, self.q[s][j] + d)

    def reset(self):
        self.neutral_active = False
        self.q = {s: dict(NEUTRAL) for s in SIDES}

    def start_neutral(self):
        self.neutral_active = True

    def tick(self, dt):
        if not self.neutral_active:
            return
        step = math.radians(NEUTRAL_SPEED_DEG_S) * dt
        done = True
        for s in SIDES:
            for j in JOINTS:
                d = NEUTRAL[j] - self.q[s][j]
                if abs(d) > 1e-4:
                    done = False
                self.q[s][j] += max(-step, min(step, d))
        if done:
            self.neutral_active = False


class ControlPanel:
    """Tk window. `app` provides: connect_real, toggle_verify, toggle_real, toggle_pause,
    emergency_stop, neutral, reset_sim, status() -> dict."""

    COLORS = {"SIM": "#2b6cb0", "CONNECTED": "#b7791f", "VERIFY": "#dd6b20",
              "REAL": "#c53030", "PAUSED": "#6b46c1", "ESTOP": "#000000"}

    def __init__(self, root, commands, app):
        self.root, self.cmd, self.app = root, commands, app
        self.sel_side, self.sel_joint = "Left", 0
        self._updating = False
        self._pressed = set()   # sliders currently held with the mouse
        root.title("SO-101 simulation -> real control")
        root.configure(padx=8, pady=8)

        self.banner = tk.Label(root, text="SIMULATION ONLY", font=("Segoe UI", 18, "bold"),
                               fg="white", bg=self.COLORS["SIM"], pady=6)
        self.banner.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.sub = tk.Label(root, text="", font=("Segoe UI", 10))
        self.sub.grid(row=1, column=0, columnspan=2, sticky="w", pady=(2, 6))

        self.scales, self.vals, self.reals, self.rowlabels, self.armstat = {}, {}, {}, {}, {}
        for col, side in enumerate(SIDES):
            fr = tk.LabelFrame(root, text=f"{side} arm", font=("Segoe UI", 11, "bold"), padx=6, pady=4)
            fr.grid(row=2, column=col, padx=4, sticky="n")
            self.armstat[side] = tk.Label(fr, text="real: not connected", anchor="w", width=46)
            self.armstat[side].grid(row=0, column=0, columnspan=4, sticky="w")
            tk.Label(fr, text="sim target", fg="#555").grid(row=1, column=2)
            tk.Label(fr, text="real (sent / measured)", fg="#555").grid(row=1, column=3)
            for r, j in enumerate(JOINTS, start=2):
                lab = tk.Label(fr, text=LABEL[j], width=14, anchor="w")
                lab.grid(row=r, column=0, sticky="w")
                lo, hi = commands.limits[side][j]
                if j == "gripper":
                    frm, to, res = 0.0, 100.0, 1.0
                else:
                    frm, to, res = math.degrees(lo), math.degrees(hi), 0.5
                sc = tk.Scale(fr, from_=frm, to=to, resolution=res, orient="horizontal", length=220,
                              showvalue=False, takefocus=0,
                              command=lambda v, s=side, jj=j: self._on_scale(s, jj, v))
                sc.bind("<ButtonPress-1>", lambda e, key=(side, j): self._pressed.add(key))
                sc.bind("<ButtonRelease-1>", lambda e, key=(side, j): self._release(key))
                sc.grid(row=r, column=1)
                val = tk.Label(fr, text="", width=9, anchor="e", font=("Consolas", 10))
                val.grid(row=r, column=2)
                real = tk.Label(fr, text="-", width=18, anchor="e", font=("Consolas", 10), fg="#555")
                real.grid(row=r, column=3)
                self.scales[(side, j)], self.vals[(side, j)] = sc, val
                self.reals[(side, j)], self.rowlabels[(side, j)] = real, lab

        bf = tk.Frame(root)
        bf.grid(row=3, column=0, columnspan=2, pady=8, sticky="ew")
        buttons = [("Connect arms (K)", app.connect_real, None),
                   ("Verify mapping (V)", app.toggle_verify, None),
                   ("ENABLE / DISABLE REAL (Enter)", app.toggle_real, "#fed7d7"),
                   ("Pause / resume (Space)", app.toggle_pause, None),
                   ("Neutral pose (C)", app.neutral, None),
                   ("Reset sim (R)", app.reset_sim, None),
                   ("Record waypoint (P)", app.record_waypoint, "#c6f6d5"),
                   ("Undo last waypoint (U)", app.undo_waypoint, None),
                   ("PLAY / STOP sequence (O)", app.toggle_play, "#c6f6d5")]
        for i, (txt, fn, bg) in enumerate(buttons):
            b = tk.Button(bf, text=txt, command=fn, padx=6, takefocus=0)
            if bg:
                b.configure(bg=bg)
            b.grid(row=i // 3, column=i % 3, padx=3, pady=3, sticky="ew")
        tk.Button(bf, text="EMERGENCY STOP (Esc)", command=app.emergency_stop, bg="#c53030", fg="white",
                  font=("Segoe UI", 12, "bold"), pady=6).grid(row=0, column=3, rowspan=3, padx=6, sticky="ns")
        self.fold_btn = tk.Button(bf, text="FOLD\nT-SHIRT (F)", command=app.fold_tshirt, bg="#2f855a", fg="white",
                                  activebackground="#276749", font=("Segoe UI", 12, "bold"), width=11, pady=6,
                                  takefocus=0)
        self.fold_btn.grid(row=0, column=4, rowspan=3, padx=(0, 6), sticky="ns")

        self.mirror_var = tk.BooleanVar(value=True)
        tk.Checkbutton(root, text="Mirror pan & wrist roll when controlling BOTH arms",
                       variable=self.mirror_var, command=self._on_mirror).grid(row=4, column=0, columnspan=2, sticky="w")
        help_txt = ("Keys (this window focused):  1-6 left-arm joints   7 8 9 0 - = right-arm joints 1-6   "
                    "TAB: Left / Right / BOTH arms\n"
                    "UP / DOWN: move joint (SHIFT = 5x)   SPACE pause   ENTER real on/off   ESC emergency stop   "
                    "C neutral   R reset   K connect   V verify\n"
                    "Teach & replay:  P record waypoint (current pose of BOTH arms)   U undo last   O play / stop   F FOLD T-SHIRT\n"
                    "3D window: Ctrl + right-drag an arm link to pull it; Ctrl + left-drag to twist it.  ESC there = emergency stop.")
        tk.Label(root, text=help_txt, justify="left", fg="#333", font=("Segoe UI", 9)).grid(
            row=5, column=0, columnspan=2, sticky="w", pady=(6, 0))

        root.bind("<KeyPress>", self._on_key)
        root.bind("<Tab>", lambda e: (self._cycle_side(), "break")[1])
        root.protocol("WM_DELETE_WINDOW", app.quit)

    # ---------- input ----------
    def _on_mirror(self):
        self.cmd.mirror = self.mirror_var.get()

    def _on_scale(self, side, j, v):
        # Tk also calls this (later) after programmatic .set(); only mouse-held sliders count.
        if self._updating or (side, j) not in self._pressed:
            return
        v = float(v)
        self.cmd.set(side, j, pct_to_grip(v) if j == "gripper" else math.radians(v))
        self.sel_side, self.sel_joint = side, JOINTS.index(j)

    def _release(self, key):
        self._pressed.discard(key)
        self.root.focus_set()  # keep keyboard control on the panel, not the slider

    def _cycle_side(self):
        order = ["Left", "Right", "Both"]
        self.sel_side = order[(order.index(self.sel_side) + 1) % 3]

    def _on_key(self, e):
        k = e.keysym
        shift = bool(e.state & 0x0001)
        if k in ("1", "2", "3", "4", "5", "6"):
            self.sel_joint = int(k) - 1
            if self.sel_side != "Both":  # 1-6 = left-arm joints (unless driving both arms)
                self.sel_side = "Left"
        elif k in ("7", "8", "9", "0", "minus", "equal"):
            self.sel_side = "Right"
            self.sel_joint = ["7", "8", "9", "0", "minus", "equal"].index(k)
        elif k in ("Up", "Down"):
            j = JOINTS[self.sel_joint]
            step = STEP_DEG_FAST if shift else STEP_DEG
            sign = 1.0 if k == "Up" else -1.0
            if j == "gripper":
                lo, hi = SIM_GRIPPER_RANGE
                delta = sign * step * 2.0 / 100.0 * (hi - lo)  # 2% (10% with shift) per press
            else:
                delta = sign * math.radians(step)
            self.cmd.nudge(self.sel_side, j, delta)
        elif k == "space":
            self.app.toggle_pause()
        elif k == "Return":
            self.app.toggle_real()
        elif k == "Escape":
            self.app.emergency_stop()
        elif k in ("r", "R"):
            self.app.reset_sim()
        elif k in ("c", "C"):
            self.app.neutral()
        elif k in ("k", "K"):
            self.app.connect_real()
        elif k in ("v", "V"):
            self.app.toggle_verify()
        elif k in ("p", "P"):
            self.app.record_waypoint()
        elif k in ("u", "U"):
            self.app.undo_waypoint()
        elif k in ("o", "O"):
            self.app.toggle_play()
        elif k in ("f", "F"):
            self.app.fold_tshirt()

    # ---------- dialogs ----------
    def confirm_enable(self, max_diff_deg):
        return messagebox.askyesnocancel(
            "Enable REAL robots?",
            "The physical arms will start following the simulation.\n\n"
            f"Largest difference between sim and real pose right now: {max_diff_deg:.0f} deg.\n\n"
            "YES  = first snap the SIMULATION to the real arms' pose (no robot motion on enable)\n"
            "NO   = keep the sim pose; the real arms move there slowly (rate-limited)\n"
            "CANCEL = stay in simulation only\n\n"
            "Keep a hand near the power switches. ESC = emergency stop.",
            icon="warning", parent=self.root)

    def info(self, title, msg):
        messagebox.showinfo(title, msg, parent=self.root)

    # ---------- display ----------
    def refresh(self, status):
        mode = status["mode"]
        texts = {"SIM": "SIMULATION ONLY", "CONNECTED": "REAL ARMS CONNECTED - NOT FOLLOWING",
                 "VERIFY": "VERIFY MAPPING - TORQUE OFF, SUPPORT THE ARMS",
                 "REAL": "REAL ROBOTS ENABLED", "PAUSED": "REAL ROBOTS PAUSED (holding)",
                 "ESTOP": "EMERGENCY STOP - commands stopped"}
        self.banner.configure(text=texts[mode], bg=self.COLORS[mode])
        if self.app.player:
            self.fold_btn.configure(text="STOP\nFOLD (F)", bg="#b7791f", activebackground="#975a16")
        else:
            self.fold_btn.configure(text="FOLD\nT-SHIRT (F)", bg="#2f855a", activebackground="#276749")
        sel = "BOTH arms" if self.sel_side == "Both" else f"{self.sel_side} arm"
        self.sub.configure(text=f"Selected: {sel}, joint {LABEL[JOINTS[self.sel_joint]]}    |    {status['note']}")

        self._updating = True
        try:
            for side in SIDES:
                st = status["arms"][side]
                self.armstat[side].configure(text=f"real: {st['text']}", fg=st["color"])
                for j in JOINTS:
                    v = self.cmd.q[side][j]
                    shown = grip_to_pct(v) if j == "gripper" else math.degrees(v)
                    if (side, j) not in self._pressed:
                        self.scales[(side, j)].set(shown)
                    unit = "%" if j == "gripper" else "°"
                    self.vals[(side, j)].configure(text=f"{shown:7.1f}{unit}")
                    sent, meas = st["sent"], st["measured"]
                    if meas is None:
                        self.reals[(side, j)].configure(text="-")
                    else:
                        s_txt = "   -  " if sent is None else f"{sent[j]:6.1f}"
                        self.reals[(side, j)].configure(text=f"{s_txt} / {meas[j]:6.1f}")
                    selected = (self.sel_side in (side, "Both")) and JOINTS[self.sel_joint] == j
                    self.rowlabels[(side, j)].configure(bg="#fefcbf" if selected else self.root.cget("bg"))
        finally:
            self._updating = False

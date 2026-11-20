# Getting started

This guide takes you from a fresh clone to (1) the simulation running, (2) the full real-arm workflow
tested with mock arms, and (3) two physical SO-101 followers safely copying the simulation.
Do the steps in order. Each one is safe without the next.

---

## 1. Install

Requirements: Python 3.10+ (tested with 3.12), a desktop session (the app opens two windows), and Linux, Windows or macOS.

```bash
git clone https://github.com/nourelshawaf/so101-sim-teleop.git
cd so101-sim-teleop
python -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Tkinter must be available. It ships with Python on Windows and macOS. On Debian/Ubuntu, run `sudo apt install python3-tk`.

## 2. Simulation only (no hardware, always safe)

```bash
python sim_teleop.py
```

Two windows open:

* **3D view (MuJoCo)**: two SO-101 arms 30 cm apart, a table, and a deformable cloth.
* **Control panel**: one slider per joint, buttons, and a status banner (green **SIM** = nothing physical connected).

Try it:

| Key | Action |
|---|---|
| `1`–`6` | Select a left-arm joint (pan, lift, elbow, wrist flex, wrist roll, gripper) |
| `7 8 9 0 - =` | Select a right-arm joint |
| `Tab` | Cycle Left → Right → Both (Both mirrors pan and wrist roll) |
| `↑` / `↓` | Move the joint 1° (`Shift`: 5°); the gripper moves 2 % (10 %) |
| `C` | Glide to the neutral pose · `R` reset the sim |
| `O` | Play the taught sequence (`fold_tshirt.json`) in the sim |
| `Space` | Pause · `Esc` emergency stop |

In the 3D view, **Ctrl + drag** an arm link to push it by hand. The arm keeps the new pose.
If the arm closes its gripper near the cloth, it grabs it.

Useful flags: `--no-cloth` (faster), `--spacing 0.4`, `--sequence other.json`, `--play-speed 20`.

## 3. Rehearse with mock arms (no hardware)

```bash
python sim_teleop.py --mock-robots
```

This runs the exact real-arm workflow from step 5 against simulated motors. Practise `K`, `V`, `ENTER` and `Esc` here
until the sequence feels natural.

## 4. Prepare the real arms (once)

1. **Calibrate both followers with LeRobot** (skip this if it is already done). The ids must be `left_follower` and `right_follower`:
   ```bash
   lerobot-calibrate --robot.type=so101_follower --robot.port=<PORT> --robot.id=left_follower
   ```
   The calibration files go to `~/.cache/huggingface/lerobot/calibration/robots/so_follower/`.
   You can also restore the backups shipped in [`calibration/`](../calibration) by copying them there.
2. **Find the serial ports.** Run `lerobot-find-port`, which tells you which port disappears when you unplug the arm.
   * Windows: `COM7` / `COM9` (the defaults).
   * Linux: `/dev/ttyACM0`, … Prefer the stable names in `/dev/serial/by-id/…`.
     Add yourself to the `dialout` group once (`sudo usermod -aG dialout $USER`, then log out and back in).
3. If one joint's range was recorded badly, re-record only that joint and keep the rest:
   ```bash
   python recalibrate_joint.py --port <PORT> --id right_follower --joint shoulder_lift
   ```

## 5. Drive the real arms

```bash
python sim_teleop.py --left-port /dev/ttyACM0 --right-port /dev/ttyACM1     # Linux example
python sim_teleop.py                                                         # Windows: COM7 / COM9
```

Follow this order every time:

1. **`K` – connect.** Torque comes on with *goal = present position*, so the arms hold where they are. The banner reads **CONNECTED**.
2. **`V` – verify mapping** (first session, or after changing anything). Torque turns **off**, so **support the arms**.
   Move one real joint at a time: the matching sim joint must move the same way.
   If a joint is mirrored, flip its sign in `DIRECTION` in [`joint_mapping.py`](../joint_mapping.py).
   Press `V` again to turn torque back on.
3. **`ENTER` – enable following.** A dialog shows the largest sim-to-real difference.
   Answer **YES** to snap the sim to the real pose first, so nothing moves on enable.
   The banner turns red: **REAL ROBOTS FOLLOWING**.
4. Move the sim (keys, sliders or mouse). The real arms follow at no more than 60 °/s, clamped to their calibrated limits.
5. Stop anytime: `Space` pauses (the arms hold), `Esc` is the emergency stop (latched; `ENTER` re-enables), and `ENTER` again turns following off.
6. **Before quitting, support the arms.** Quitting disconnects them and LeRobot switches torque off.

A red **FAULT** on an arm means a communication error. That arm gets no further commands. Press `K` to reconnect it.

## 6. Teach and replay a task (t-shirt fold)

1. `K`, then `V` (torque off, support the arms).
2. Move the **real** arms by hand to the first key pose and press `P`. Repeat for every key pose:
   approach above the corners → down → close grippers → lift → carry over → lower → open → retreat.
   Record one pose **before and after** every gripper close or open. `U` deletes the last waypoint.
   Each waypoint is saved immediately to `fold_tshirt.json`.
3. Press `V` (torque on, holding). Press `O` with following **off** to preview the fold in the sim only.
4. Press the green **FOLD T-SHIRT** button (or `F`). If the arms aren't following yet, it asks first, then folds.
   While it runs, the button reads **STOP FOLD**.

Tune the fold in the JSON: `"hold"` (seconds per waypoint) and `--play-speed` (°/s of the fastest joint).

## 7. Re-render the demo video

```bash
MUJOCO_GL=egl python tools/record_demo.py        # -> docs/media/demo.mp4 (offscreen, ~2 min)
```

On Windows or macOS, drop `MUJOCO_GL=egl`. The video replays whatever is currently in `fold_tshirt.json`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `No module named 'tkinter'` | `sudo apt install python3-tk` |
| `Permission denied: /dev/ttyACM0` | Join the `dialout` group, then log out and back in |
| Connect fails / FAULT right away | Wrong port or id, or the calibration file is missing (step 4). Check that the other app isn't holding the port |
| A real joint moves opposite to the sim | Flip that joint in `DIRECTION` in `joint_mapping.py` and re-check with `V` |
| A gripper opens when it should close | Set `GRIPPER_INVERT[side] = True` in `joint_mapping.py` |
| Real arm stops short of the sim pose | Expected: it is clamped to the calibrated range minus 3°. Re-record that joint (step 4.3) |
| `Attach conflict` warnings in `MUJOCO_LOG.TXT` | Harmless. The bimanual scene keeps its own 2 ms timestep |
| `framebuffer width` error in `record_demo.py` | Use the script as shipped (it enlarges the offscreen buffer) |

# Roadmap: ideas for v0.2 and beyond

This project lets two SO-101 arms copy their MuJoCo digital twins, and it can teach and replay a bimanual task.
Below are ideas for the next versions, each with a pointer to where it lives in the code.
**Pick one, open an issue saying you're on it, and send a PR.** See [CONTRIBUTING.md](CONTRIBUTING.md).

Labels: 🟢 good first issue (an evening) · 🟡 medium (a weekend) · 🔴 big (multi-week, discuss in an issue first) ·
⚠️ touches real-arm motion, so it needs a hardware test or a reviewer who has the arms.

---

## v0.2: polish and developer experience

| # | Idea | Where | Size |
|---|---|---|---|
| 1 | **Unit tests** for `joint_mapping` (round trip, gripper invert, limits) and `motion_sequence` (`Player` timing, smoothstep endpoints, empty sequence) with `pytest` | new `tests/` | 🟢 |
| 2 | **Headless integration test**: `SyncManager` + `MockArm`. Check that connect → HOLDING, follow respects the 60 °/s cap, the e-stop latches, and a fault isolates one arm | `tests/`, `robot_sync.py` | 🟢 |
| 3 | **GitHub Actions CI**: ruff + pytest on Linux with `MUJOCO_GL=egl`, plus a smoke run of `tools/record_demo.py` | `.github/workflows/` | 🟢 |
| 4 | **OS-aware default ports**: `COM7`/`COM9` on Windows, auto-pick from `/dev/serial/by-id/*` on Linux, and print what was found | `sim_teleop.py` (`main`) | 🟢 |
| 5 | **Config file** (`config.yaml`) for ports, ids, spacing, speeds and sequence, with CLI flags overriding it | `sim_teleop.py` | 🟢 |
| 6 | **Silence the `Attach conflict` warnings** by aligning the child model's options before `attach_body` | `sim_teleop.py` (`build_model`) | 🟢 |
| 7 | **`pyproject.toml`** with pinned, tested versions and a `so101-teleop` console script | repo root | 🟢 |
| 8 | **Playback speed slider** and a progress bar (waypoint *k / n*, time left) in the control panel | `simulation_controller.py` | 🟢 |
| 9 | **Waypoint editor**: list the waypoints, jump to one in the sim, edit `hold`, insert/delete/reorder, rename | `motion_sequence.py`, `simulation_controller.py` | 🟡 |
| 10 | **Sequence library**: a dropdown of `sequences/*.json` (fold, hand over, wave…) instead of one file | `sim_teleop.py`, `simulation_controller.py` | 🟡 |

## v0.3: safety and robustness ⚠️

| # | Idea | Where | Size |
|---|---|---|---|
| 11 | **Arm-to-arm collision guard**: if MuJoCo reports contact between `left_*` and `right_*` geoms (or the table), pause following and show why | `sim_teleop.py` (`tick`) | 🟡 ⚠️ |
| 12 | **Tracking-error watchdog**: if `measured` lags `last_sent` by more than X° for more than Y ms (blocked or overloaded), drop to HOLDING | `robot_sync.py` (`_follow_step`) | 🟡 ⚠️ |
| 13 | **Current/temperature monitoring**: read `Present_Current` and `Present_Temperature`, show them in the panel, fault on overload | `real_robot_controller.py`, `robot_sync.py` | 🟡 ⚠️ |
| 14 | **Gripper force limit**: cap the gripper torque while closing on cloth so it doesn't stall the servo | `real_robot_controller.py` | 🟡 ⚠️ |
| 15 | **Hardware e-stop**: map a USB foot pedal or keyboard key that works even when the windows don't have focus | `sim_teleop.py` | 🟡 ⚠️ |
| 16 | **Session log**: write the states, targets, measured poses and faults to CSV for debugging | `robot_sync.py` | 🟢 |

## v0.4: new ways to drive the arms

| # | Idea | Where | Size |
|---|---|---|---|
| 17 | **Cartesian teleop with IK** (e.g. [mink](https://github.com/kevinzakka/mink)): move the gripper in x/y/z/pitch instead of joint by joint | new `ik.py`, panel | 🔴 |
| 18 | **Gamepad / SpaceMouse input** as another source for `JointCommands` | `simulation_controller.py` | 🟡 |
| 19 | **SO-101 leader arms** as input. The sim stays in the middle, so the existing safety layer still applies | new input module | 🟡 ⚠️ |
| 20 | **Click-to-place** in the 3D view: pick a point on the table and the arm reaches it (needs #17) | `sim_teleop.py` | 🔴 |

## v1.0: autonomy and learning

| # | Idea | Where | Size |
|---|---|---|---|
| 21 | **Record replays as a LeRobot dataset** (joint states, actions, optional camera) for imitation learning | new `record_dataset.py` | 🟡 |
| 22 | **Train and run a policy** (ACT / Diffusion Policy) on that dataset, executed through the same sim → safety → real path | new `policy/` | 🔴 |
| 23 | **Vision-based shirt localisation**: an overhead camera finds the shirt and offsets the taught waypoints (closed-loop start pose) | new `perception/` | 🔴 |
| 24 | **Better cloth model**: friction-based grasping instead of welds, and tune the flex parameters against a real t-shirt | `sim_teleop.py` (`CLOTH`, `update_grasp`) | 🔴 |
| 25 | **System identification**: fit the actuator gains and damping so the sim and the real arm move with the same timing | `robotstudio_so101/so101.xml`, new `sysid/` | 🔴 |
| 26 | **ROS 2 bridge**: publish `/joint_states` and accept trajectory goals, so the twin plugs into MoveIt and RViz | new `ros2/` | 🔴 |
| 27 | **Support other arms** (SO-100, Koch, single-arm mode) by moving arm definitions into config | `joint_mapping.py`, `sim_teleop.py` | 🟡 |

## Non-code contributions (also very welcome)

* 🎥 A video of the **real arms folding** a shirt, for the README.
* 📖 Translate [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md).
* 🧪 Report how the app runs on your OS, Python version, or calibration (open a *hardware report* issue).
* 👕 Share taught sequences (`sequences/*.json`) for other tasks, with the table setup you used.

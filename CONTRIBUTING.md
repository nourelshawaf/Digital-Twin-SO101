# Contributing

Thanks for helping! You don't need SO-101 hardware to contribute: most of the code runs with
`--mock-robots` or fully offscreen.

## 1. Find something to work on

* Browse [ROADMAP.md](ROADMAP.md). 🟢 items are good first issues.
* Open (or comment on) an issue saying what you'll take, so two people don't do the same work.
* For 🔴 items, describe your approach in the issue before writing a lot of code.

## 2. Set up

```bash
git clone https://github.com/nourelshawaf/so101-sim-teleop.git && cd so101-sim-teleop
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python sim_teleop.py --mock-robots          # full workflow, no hardware
MUJOCO_GL=egl python tools/record_demo.py   # headless render, a quick end-to-end check
```

See [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) for details and [docs/REPORT.md](docs/REPORT.md) for the architecture.

## 3. Ground rules for code

* **The sim is the master.** New input methods edit `JointCommands`. Nothing writes to the motors except `ArmSync` in
  `robot_sync.py`, so every motion passes through the clamp, the rate limit, the e-stop and pause.
* **Never weaken a safety rule without discussing it first.** That covers the safe torque-enable, the confirmation on enable,
  the latched e-stop, the 60 °/s cap, calibrated limits, FAULT isolation, and starting in sim-only mode.
* **No blocking I/O on the main (Tk/MuJoCo) thread.** Bus access belongs in the arm's own thread.
* **Units:** sim = radians and real = degrees (gripper 0–100). Convert only in `joint_mapping.py`.
* Keep modules small, with a docstring at the top that explains *why*. Match the existing style (ruff, 120 columns).
* Add or update tests for logic that can run without hardware.

## 4. Pull requests

* Keep each PR focused on one roadmap item or bug.
* Fill in the PR template. If a PR changes real-arm behaviour, it must say whether it was **tested on hardware**.
  If it wasn't, a maintainer with the arms will test it before merging.
* Update the docs (README, GETTING_STARTED, ROADMAP) when the behaviour or the keys change.
* For visual changes, attach a screenshot or a short clip, or re-render `docs/media/demo.mp4`.

## 5. Reporting bugs

Use the issue templates. For real-arm problems, include your OS, port names, the LeRobot version, the calibration
json (it has no secrets), and the console output around the FAULT.

Be kind and constructive. We're all here to make robots fold laundry. 👕🤖

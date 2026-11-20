## What
<!-- Roadmap item # / issue #, and a short description -->

## How tested
- [ ] `python sim_teleop.py` (sim only)
- [ ] `python sim_teleop.py --mock-robots`
- [ ] Tests / `tools/record_demo.py` still pass
- [ ] Tested on real SO-101 arms (describe below), or **not tested on hardware**

## Safety checklist (if real-arm motion is affected)
- [ ] Motor writes still go only through `ArmSync` (clamp + rate limit + e-stop + pause)
- [ ] No safety rule weakened (safe torque-enable, enable confirmation, latched e-stop, limits, FAULT isolation)
- [ ] No blocking serial I/O on the main thread

## Docs
- [ ] README / GETTING_STARTED / ROADMAP updated if behaviour or keys changed

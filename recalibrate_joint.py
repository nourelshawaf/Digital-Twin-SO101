"""
Re-record the range of motion of ONE joint (or a few) of an SO-101 follower, keeping everything else.

Only range_min / range_max of the chosen joint(s) change. The homing offsets and all other
joints stay exactly as they are, so joints that already match the sim are not disturbed.
A timestamped backup of the calibration file is written first.

  python recalibrate_joint.py --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14029771-if00 \
                              --id right_follower --joint shoulder_lift

Run it in a terminal (it needs ENTER from you) and with sim_teleop.py closed (it uses the port).
Torque is OFF on the whole arm while it runs: support the arm.
"""
import argparse
import shutil
import time

from joint_mapping import JOINTS


def load_lerobot():
    try:
        from lerobot.robots.so_follower import SO101Follower as Follower
        from lerobot.robots.so_follower import SO101FollowerConfig as Cfg
    except ImportError:
        from lerobot.robots.so101_follower import SO101Follower as Follower
        from lerobot.robots.so101_follower import SO101FollowerConfig as Cfg
    return Follower, Cfg

TICKS_TO_DEG = 360.0 / 4095


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", required=True, help="calibration id, e.g. right_follower")
    ap.add_argument("--joint", nargs="+", default=["shoulder_lift"], choices=JOINTS)
    ap.add_argument("--seconds", type=float, default=0,
                    help="timed mode, no keyboard needed: wait 10 s, record for this many seconds, then save "
                         "automatically ONLY if every joint's new range is wider than its old one")
    args = ap.parse_args()

    Follower, Cfg = load_lerobot()
    robot = Follower(Cfg(port=args.port, id=args.id, use_degrees=True))
    if not robot.calibration:
        raise SystemExit(f"No calibration file for id '{args.id}' ({robot.calibration_fpath}). "
                         "Run the full LeRobot calibration instead.")
    bus = robot.bus
    bus.connect()
    try:
        motor_homing = bus.sync_read("Homing_Offset", normalize=False)
        wrong = [j for j in JOINTS if motor_homing[j] != robot.calibration[j].homing_offset]
        if wrong:
            raise SystemExit(f"The motors on {args.port} don't match '{args.id}' (homing differs on {wrong}). "
                             "Wrong arm/port? Nothing was changed.")

        print(f"\nArm '{args.id}' on {args.port}. Current range:")
        for j in args.joint:
            c = robot.calibration[j]
            print(f"  {j:14s} {c.range_min}..{c.range_max}  = {(c.range_max - c.range_min) * TICKS_TO_DEG:.0f} deg")

        if args.seconds > 0:
            bus.disable_torque()
            print("\nTorque OFF. Support the arm. Recording starts in 10 s.", flush=True)
            time.sleep(10)
            print(f"RECORDING {args.seconds:.0f} s: move {', '.join(args.joint)} to BOTH ends, slowly.", flush=True)
            pos = bus.sync_read("Present_Position", args.joint, normalize=False, num_retry=5)
            mins, maxes = dict(pos), dict(pos)
            t_end = time.time() + args.seconds
            while time.time() < t_end:
                pos = bus.sync_read("Present_Position", args.joint, normalize=False, num_retry=5)
                for j in args.joint:
                    mins[j], maxes[j] = min(mins[j], pos[j]), max(maxes[j], pos[j])
                time.sleep(0.02)
            print("Recording done.", flush=True)
        else:
            input("\nTorque will go OFF on the whole arm. SUPPORT THE ARM, then press ENTER (Ctrl+C to cancel)... ")
            bus.disable_torque()
            print(f"\nMove {', '.join(args.joint)} slowly through its FULL range, all the way to BOTH mechanical ends.\n"
                  "Watch MIN and MAX grow. Then press ENTER.\n")
            mins, maxes = bus.record_ranges_of_motion(args.joint)

        print("\nResult:")
        for j in args.joint:
            c = robot.calibration[j]
            print(f"  {j:14s} old {c.range_min}..{c.range_max} ({(c.range_max - c.range_min) * TICKS_TO_DEG:.0f} deg)"
                  f"  ->  new {mins[j]}..{maxes[j]} ({(maxes[j] - mins[j]) * TICKS_TO_DEG:.0f} deg)")
        if args.seconds > 0:
            narrower = [j for j in args.joint
                        if maxes[j] - mins[j] <= robot.calibration[j].range_max - robot.calibration[j].range_min]
            if narrower:
                print(f"NOT saved: the new range is not wider than the old one for {narrower}. Nothing was changed.")
                return
        elif input("\nSave this? [y/N] ").strip().lower() != "y":
            print("Not saved. Nothing was changed.")
            return

        backup = f"{robot.calibration_fpath}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
        shutil.copy2(robot.calibration_fpath, backup)
        for j in args.joint:
            robot.calibration[j].range_min = int(mins[j])
            robot.calibration[j].range_max = int(maxes[j])
        bus.write_calibration({j: robot.calibration[j] for j in args.joint}, cache=False)
        robot._save_calibration()
        print(f"Saved to {robot.calibration_fpath} (backup: {backup}).")
        print("Now restart sim_teleop.py, connect (K) and check that joint with Verify (V).")
    finally:
        bus.disconnect(disable_torque=False)  # torque is already off (or was never turned on)


if __name__ == "__main__":
    main()

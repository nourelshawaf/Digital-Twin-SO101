"""
Thin wrapper around one physical SO-101 follower (LeRobot SOFollower), plus a mock for testing.

All methods here talk to the serial bus, so they must only be called from the arm's own
sync thread (robot_sync.ArmSync). Nothing here runs at import time and nothing moves the arm
unless write_goal() is called.

Safety detail: LeRobot re-enables torque in several places (connect/configure, torque_disabled
blocks). When torque comes on, a Feetech servo drives to whatever Goal_Position is stored in it,
which can be stale (e.g. after you moved the arm by hand). We wrap the bus's enable_torque so it
first copies Present_Position into Goal_Position: torque always comes on holding the current pose.
"""
import math
import time

from joint_mapping import BODY_JOINTS, JOINTS, real_limits_from_calibration


class RealArm:
    def __init__(self, side, port, robot_id):
        self.side, self.port, self.robot_id = side, port, robot_id
        self.robot = None
        self.limits = None
        self.torque_on = False

    @property
    def connected(self):
        return self.robot is not None and self.robot.is_connected

    def connect(self):
        try:
            from lerobot.robots.so_follower import SO101Follower as Follower
            from lerobot.robots.so_follower import SO101FollowerConfig as Cfg
        except ImportError:
            from lerobot.robots.so101_follower import SO101Follower as Follower
            from lerobot.robots.so101_follower import SO101FollowerConfig as Cfg
        cfg = Cfg(port=self.port, id=self.robot_id, use_degrees=True)
        robot = Follower(cfg)
        self._install_safe_torque(robot.bus)
        robot.connect(calibrate=False)  # never start the interactive calibration from here
        self.robot = robot
        self.torque_on = True            # LeRobot connect() leaves torque on (holding pose)
        self.limits = real_limits_from_calibration(robot.calibration)

    @staticmethod
    def _install_safe_torque(bus):
        original = bus.enable_torque

        def enable_torque_holding_pose(*args, **kwargs):
            present = bus.sync_read("Present_Position")
            bus.sync_write("Goal_Position", present)
            return original(*args, **kwargs)

        bus.enable_torque = enable_torque_holding_pose

    def read(self):
        pos = self.robot.bus.sync_read("Present_Position", num_retry=2)
        return {j: float(pos[j]) for j in JOINTS}

    def write_goal(self, goal):
        self.robot.bus.sync_write("Goal_Position", {j: float(goal[j]) for j in JOINTS})

    def set_torque(self, on):
        if on:
            self.robot.bus.enable_torque()
        else:
            self.robot.bus.disable_torque()
        self.torque_on = on

    def disconnect(self):
        if self.robot is not None:
            try:
                self.robot.disconnect()  # LeRobot default: torque off on disconnect (arm goes limp)
            finally:
                self.robot = None
                self.torque_on = False


class MockArm:
    """Behaves like RealArm without hardware (for --mock-robots testing)."""

    def __init__(self, side, port, robot_id):
        self.side, self.port, self.robot_id = side, port, robot_id
        self._connected = False
        self.torque_on = False
        self.limits = {j: (-100.0, 100.0) for j in BODY_JOINTS}
        self.limits["wrist_roll"] = (-150.0, 150.0)
        self.limits["gripper"] = (1.0, 99.0)
        self._pos = {j: 0.0 for j in JOINTS}
        self._pos.update(shoulder_lift=-90.0, elbow_flex=90.0, gripper=10.0)  # a folded rest pose
        self._goal = dict(self._pos)
        self._t = time.time()
        self.fail_after = None  # set to a number of writes to simulate a comms failure

    @property
    def connected(self):
        return self._connected

    def connect(self):
        time.sleep(0.3)
        self._connected = True
        self.torque_on = True
        self._goal = dict(self._pos)

    def _advance(self):
        now = time.time()
        dt, self._t = now - self._t, now
        if not self.torque_on:
            return
        k = 1.0 - math.exp(-dt / 0.08)  # servo lag
        for j in JOINTS:
            self._pos[j] += k * (self._goal[j] - self._pos[j])

    def read(self):
        if not self._connected:
            raise ConnectionError("mock not connected")
        self._advance()
        return dict(self._pos)

    def write_goal(self, goal):
        if self.fail_after is not None:
            self.fail_after -= 1
            if self.fail_after < 0:
                raise ConnectionError(f"Failed to write Goal_Position on {self.port} (simulated)")
        self._goal = dict(goal)

    def set_torque(self, on):
        if on:
            self._goal = dict(self._pos)
        self.torque_on = on

    def move_by_hand(self, joint, value):
        """Test helper: what a person moving the limp arm would do."""
        self._pos[joint] = value

    def disconnect(self):
        self._connected = False
        self.torque_on = False

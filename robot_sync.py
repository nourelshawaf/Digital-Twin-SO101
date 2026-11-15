"""
Simulation -> real synchronisation.

One background thread per physical arm (ArmSync). Each thread owns its serial port, so the
MuJoCo render/physics loop never blocks on motor I/O, and a failing arm cannot stall the other.

Per cycle (default 30 Hz), when FOLLOWING:
  target (real units, set by the sim loop) -> clamp to calibrated limits
  -> rate limit (max deg/s) starting from the arm's measured pose -> write Goal_Position

Safety rules enforced here:
  - nothing is written unless the arm is FOLLOWING, not paused and the e-stop is not latched
  - e-stop latches: every arm drops to HOLDING (torque stays on, holding its pose) until you
    re-enable following explicitly
  - any communication error puts that arm in FAULT and stops all writes to it
"""
import queue
import threading
import time

from joint_mapping import JOINTS, clamp_real

DISCONNECTED, CONNECTING, HOLDING, VERIFY, FOLLOWING, FAULT = (
    "DISCONNECTED", "CONNECTING", "HOLDING", "VERIFY", "FOLLOWING", "FAULT")

MAX_SPEED_DEG_S = 60.0      # body joints
MAX_GRIPPER_PCT_S = 120.0   # gripper (0..100 scale)
DEADBAND = 0.05             # don't re-send changes smaller than this


class ArmSync(threading.Thread):
    def __init__(self, arm, estop_event, pause_event, hz=30.0):
        super().__init__(daemon=True, name=f"sync-{arm.side}")
        self.arm = arm
        self.estop = estop_event
        self.paused = pause_event
        self.period = 1.0 / hz
        self.state = DISCONNECTED
        self.error = ""
        self.measured = None      # latest real pose (real units)
        self.last_sent = None
        self._target = None
        self._lock = threading.Lock()
        self._cmds = queue.Queue()
        self._quit = threading.Event()

    # ---- called from the UI / sim thread (non-blocking) ----
    def request(self, cmd, value=None):
        self._cmds.put((cmd, value))

    def set_target(self, target):
        with self._lock:
            self._target = dict(target)

    def shutdown(self):
        self._quit.set()

    # ---- worker ----
    def _fault(self, msg):
        self.state = FAULT
        self.error = msg
        self.last_sent = None
        print(f"[{self.arm.side} arm {self.arm.port}] FAULT: {msg} -- no more commands sent to this arm.")

    def _process_cmd(self, cmd, value):
        if cmd == "connect":
            self.state, self.error = CONNECTING, ""
            if self.arm.connected:  # reconnect after a fault: start from a fresh connection
                try:
                    self.arm.disconnect()
                except Exception:
                    pass
            self.arm.connect()
            self.measured = self.arm.read()
            self.state = HOLDING
        elif cmd == "verify":
            if not self.arm.connected or self.state == FAULT:
                return
            if value and self.state in (HOLDING, FOLLOWING):
                self.state = HOLDING
                self.arm.set_torque(False)
                self.state = VERIFY
            elif not value and self.state == VERIFY:
                self.arm.set_torque(True)  # comes on holding the current pose
                self.state = HOLDING
        elif cmd == "follow":
            if value and self.state == HOLDING and not self.estop.is_set():
                self.last_sent = None
                self.state = FOLLOWING
            elif not value and self.state == FOLLOWING:
                self.state = HOLDING
        elif cmd == "disconnect":
            self.state = DISCONNECTED
            self.arm.disconnect()

    def run(self):
        while not self._quit.is_set():
            t0 = time.time()
            try:
                while True:
                    try:
                        cmd, value = self._cmds.get_nowait()
                    except queue.Empty:
                        break
                    if self.state == FAULT and cmd not in ("disconnect", "connect"):
                        continue
                    self._process_cmd(cmd, value)

                if self.estop.is_set() and self.state == FOLLOWING:
                    self.state = HOLDING

                if self.arm.connected and self.state in (HOLDING, VERIFY, FOLLOWING):
                    self.measured = self.arm.read()
                    if self.state == FOLLOWING and not self.paused.is_set() and not self.estop.is_set():
                        self._follow_step()
                    else:
                        self.last_sent = None
            except Exception as e:  # noqa: BLE001 -- any bus error must stop this arm
                self._fault(f"{type(e).__name__}: {e}")
            time.sleep(max(0.0, self.period - (time.time() - t0)))
        try:
            self.arm.disconnect()
        except Exception:
            pass

    def _follow_step(self):
        with self._lock:
            target = None if self._target is None else dict(self._target)
        if target is None or self.measured is None:
            return
        target = clamp_real(target, self.arm.limits)
        if self.last_sent is None:
            self.last_sent = dict(self.measured)
        new = {}
        for j in JOINTS:
            step = (MAX_GRIPPER_PCT_S if j == "gripper" else MAX_SPEED_DEG_S) * self.period
            d = target[j] - self.last_sent[j]
            new[j] = self.last_sent[j] + max(-step, min(step, d))
        if max(abs(new[j] - self.last_sent[j]) for j in JOINTS) < DEADBAND:
            return
        if self.estop.is_set() or self.paused.is_set():  # re-check right before writing
            return
        self.arm.write_goal(new)
        self.last_sent = new


class SyncManager:
    """Both arms + global safety state."""

    def __init__(self, arms, hz=30.0):
        self.estop_event = threading.Event()
        self.pause_event = threading.Event()
        self.arms = {side: ArmSync(arm, self.estop_event, self.pause_event, hz)
                     for side, arm in arms.items()}
        for s in self.arms.values():
            s.start()
        self.sync_enabled = False

    def state(self, side):
        return self.arms[side].state

    def any_connected(self):
        return any(a.arm.connected for a in self.arms.values())

    def all_ready(self):
        return all(a.state in (HOLDING, FOLLOWING) for a in self.arms.values())

    def connect(self):
        for a in self.arms.values():
            if a.state in (DISCONNECTED, FAULT):
                a.request("connect")

    def set_verify(self, on):
        if on:
            self.disable_sync()
        for a in self.arms.values():
            a.request("verify", on)

    def enable_sync(self):
        self.estop_event.clear()
        self.pause_event.clear()
        self.sync_enabled = True
        for a in self.arms.values():
            a.request("follow", True)

    def disable_sync(self):
        self.sync_enabled = False
        for a in self.arms.values():
            a.request("follow", False)

    def emergency_stop(self):
        self.estop_event.set()   # threads stop writing immediately, before the next write
        self.sync_enabled = False
        for a in self.arms.values():
            a.request("follow", False)
        print("EMERGENCY STOP: no more commands are being sent. Arms hold their current pose.")

    def toggle_pause(self):
        if self.pause_event.is_set():
            self.pause_event.clear()
        else:
            self.pause_event.set()
        return self.pause_event.is_set()

    def set_targets(self, targets):
        for side, t in targets.items():
            self.arms[side].set_target(t)

    def shutdown(self):
        self.emergency_stop()
        for a in self.arms.values():
            a.shutdown()
        for a in self.arms.values():
            a.join(timeout=3.0)

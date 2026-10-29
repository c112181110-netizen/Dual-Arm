"""B→C state machine; no ROS dependency. All motion goes through the existing adapter."""
import threading
import time


def above(pose, height):
    result = list(pose)
    result[2] += height
    return result


class Arm2Workflow:
    def __init__(self, io, cfg, occupied, cancel=None, on_state=None):
        self.io, self.cfg = io, cfg
        self.occupied = list(occupied)
        self.cancel = cancel or threading.Event()
        self.on_state = on_state or (lambda status: None)
        self.holding, self.b_locked, self.state = 'empty', False, 'IDLE'
        self.slot = 0

    def transition(self, state):
        if self.cancel.is_set():
            raise InterruptedError('Canceled or safety interlock lost')
        self.io.guard()
        self.state = state
        self.emit()

    def emit(self):
        self.on_state(dict(state=self.state, holding=self.holding, b_locked=self.b_locked,
                           occupied=list(self.occupied), slot=self.slot))

    def wait_zone(self, predicate, label):
        deadline = time.monotonic() + self.cfg['zone_wait_timeout']
        while time.monotonic() < deadline:
            self.io.guard()
            if self.cancel.is_set():
                raise InterruptedError('Canceled while waiting for ' + label)
            z = self.io.zone()
            if not z.success:
                raise RuntimeError('Invalid vision: ' + z.message)
            if predicate(z):
                return z
            if self.cancel.wait(self.cfg['zone_poll_period']):
                raise InterruptedError('Canceled while waiting for ' + label)
        raise TimeoutError('Timed out waiting for ' + label)

    def home(self):
        self.io.move_joints(self.cfg['home_joints'])
        if self.cancel.wait(self.cfg['home_settle']):
            raise InterruptedError('Canceled during camera settling')

    def run(self):
        try:
            if all(self.occupied):
                self.transition('FULL')
                return
            self.slot = self.occupied.index(False) + 1
            expected = sum(self.occupied)
            b, c = self.cfg['b_pick'], self.cfg['c_slots'][self.slot - 1]
            ab, ac = above(b, self.cfg['approach_height']), above(c, self.cfg['approach_height'])
            self.transition('CHECK_B_READY')
            self.wait_zone(lambda z: z.b_has_object and z.b_arm_clear and z.c_object_count == expected,
                           'occupied, clear B and consistent C count')
            # This confirms the ENTIRE saved map and target vacancy, not just object count.
            self.transition('VERIFY_LAYOUT')
            self.io.verify_layout(self.occupied)
            self.io.suction('verify_layout')
            self.transition('PREFLIGHT')
            self.io.setup_scene()
            for p in (ab, b, ac, c):
                self.io.check_ik(p)
            self.home()
            self.wait_zone(lambda z: z.b_has_object and z.b_arm_clear and z.c_object_count == expected,
                           'B clear immediately before reservation')
            self.b_locked = True
            self.transition('B_RESERVED')
            self.io.move_pose(ab)
            self.transition('DESCEND_B')
            self.io.move_pose(b, linear=True)
            self.holding = 'unknown'
            self.transition('GRASP_B')
            self.io.suction('grasp')
            self.holding = 'held'
            self.transition('ATTACH')
            self.io.attach_disk()
            self.transition('LIFT_B')
            self.io.move_pose(ab, linear=True)
            self.transition('HOME_WITH_OBJECT')
            self.home()
            self.transition('VERIFY_B_EMPTY')
            self.wait_zone(lambda z: not z.b_has_object and z.b_arm_clear, 'B empty and arm out')
            self.b_locked = False
            self.transition('B_EMPTY')
            self.transition('APPROACH_C')
            self.io.move_pose(ac)
            self.transition('DESCEND_C')
            self.io.move_pose(c, linear=True)
            self.holding = 'unknown'
            self.transition('RELEASE_C')
            self.io.suction('release')
            self.holding = 'empty'
            self.transition('DETACH')
            self.io.detach_disk()
            self.transition('LIFT_C')
            self.io.move_pose(ac, linear=True)
            self.io.record_slot(c, self.slot)
            self.transition('RETURN_HOME')
            self.home()
            self.transition('VERIFY_C')
            self.wait_zone(lambda z: z.c_object_count == expected + 1, 'C count increased by one')
            expected_map = list(self.occupied)
            expected_map[self.slot - 1] = True
            self.io.verify_slot(expected_map, self.slot)
            self.io.suction('verify_slot')
            self.occupied[self.slot - 1] = True
            self.transition('FULL' if all(self.occupied) else 'SUCCEEDED')
        except Exception:
            self.state = 'FAULT'
            self.emit()  # Preserve B reservation and uncertain tool/slot; never auto-release.
            raise

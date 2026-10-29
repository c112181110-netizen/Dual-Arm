"""Phase 1 state machine. External vision/firmware remain black-box interfaces."""
import math
import threading
import time


class Arm1Workflow:
    def __init__(self, io, config, cancel=None, on_state=None):
        self.io, self.cfg = io, config
        self.cancel = cancel or threading.Event()
        self.on_state = on_state or (lambda state: None)
        self.holding = 'empty'
        self.b_locked = False
        self.state = 'IDLE'

    def transition(self, state):
        if self.cancel.is_set():
            raise InterruptedError('Task canceled')
        self.state = state
        self.on_state({'state': state, 'holding': self.holding, 'b_locked': self.b_locked})

    def wait(self, duration):
        if self.cancel.wait(duration):
            raise InterruptedError('Task canceled')

    def wait_zone(self, predicate, label):
        deadline = time.monotonic() + self.cfg['zone_wait_timeout']
        while time.monotonic() < deadline:
            if self.cancel.is_set():
                raise InterruptedError('Task canceled')
            result = self.io.zone()
            if not result.success:
                raise RuntimeError(f'Invalid vision result during {label}: {result.message}')
            if predicate(result):
                return result
            self.wait(self.cfg['zone_poll_period'])
        raise TimeoutError(f'Timed out waiting for {label}')

    def run(self):
        try:
            self.transition('INITIALIZING')
            self.io.setup_scene()
            self.io.move_joints(self.cfg['home_joints'])
            self.wait(self.cfg['home_settle'])
            self.transition('ARM1_STATE0_WAIT_A')
            self.wait_zone(lambda z: z.a_has_object, 'A object')
            self.transition('ARM1_STATE0_QUERY_TARGET')
            target = self.io.pick()
            if not target.success:
                raise RuntimeError(f'Vision target failed: {target.message}')
            xyz = [target.x, target.y, target.z]
            if not all(math.isfinite(v) for v in xyz):
                raise ValueError('Vision returned a non-finite target')
            if any(not low <= v <= high for low, v, high in zip(self.cfg['a_min'], xyz, self.cfg['a_max'])):
                raise ValueError(f'Vision target outside configured A bounds: {xyz}')
            if abs(target.z - self.cfg['pick_z']) > self.cfg['height_tolerance']:
                raise ValueError('Vision height differs from the calibrated fixed pick height')
            xyz[2] = self.cfg['pick_z']
            above_a = [xyz[0], xyz[1], xyz[2] + self.cfg['approach_height']]
            self.transition('ARM1_STATE1_APPROACH_A')
            self.io.move_pose(above_a)
            self.transition('ARM1_STATE1_DESCEND_A')
            self.io.move_pose(xyz, linear=True)
            # Operator may actuate suction before confirming; cancellation in
            # this window cannot be treated as an empty gripper.
            self.holding = 'unknown'
            self.transition('ARM1_STATE1_WAIT_SUCTION_ON')
            self.io.suction('grasp')
            self.holding = 'held'
            self.transition('ARM1_STATE1_ATTACH')
            self.io.attach_disk()
            self.transition('ARM1_STATE1_LIFT_A')
            self.io.move_pose(above_a, linear=True)
            self.transition('ARM1_STATE1_HOME_HOLDING')
            self.io.move_joints(self.cfg['home_joints'])
            self.wait(self.cfg['home_settle'])
            self.transition('ARM1_STATE1_WAIT_B')
            self.wait_zone(lambda z: not z.b_has_object and z.b_arm_clear, 'empty, clear B')
            # This coordinator is the single owner of both arms' zone locks.
            # Phase 1 has no arm2 executor; independent controllers must not run.
            self.b_locked = True
            self.transition('ARM1_STATE2_B_RESERVED')
            self.io.clear_placed_disk()
            b = list(self.cfg['b_place'])
            above_b = [b[0], b[1], b[2] + self.cfg['approach_height']]
            self.transition('ARM1_STATE2_APPROACH_B')
            self.io.move_pose(above_b)
            self.transition('ARM1_STATE2_DESCEND_B')
            self.io.move_pose(b, linear=True)
            self.holding = 'unknown'
            self.transition('ARM1_STATE2_WAIT_SUCTION_OFF')
            self.io.suction('release')
            self.holding = 'empty'
            self.transition('ARM1_STATE2_DETACH')
            self.io.detach_disk()
            self.transition('ARM1_STATE3_LIFT_B')
            self.io.move_pose(above_b, linear=True)
            self.io.record_placed_disk(b)
            self.transition('ARM1_STATE3_HOME')
            self.io.move_joints(self.cfg['home_joints'])
            self.wait(self.cfg['home_settle'])
            self.transition('ARM1_STATE3_VERIFY_B')
            self.wait_zone(lambda z: z.b_has_object and z.b_arm_clear, 'placed object and arm clear of B')
            self.b_locked = False
            self.transition('SUCCEEDED')
        except Exception:
            self.state = 'FAULT'
            self.on_state({'state': self.state, 'holding': self.holding, 'b_locked': self.b_locked})
            raise

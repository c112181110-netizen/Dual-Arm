import threading
from types import SimpleNamespace as S
import pytest
from dual_arm_control.arm2_workflow import Arm2Workflow
from dual_arm_control.arm2_config import validate_arm2, validate_journal

CFG = dict(home_joints=[0., 0., 1.57, 1.57], home_settle=0.001, zone_wait_timeout=0.01,
           zone_poll_period=0.001, approach_height=0.03, b_pick=[.15, -.04, .02, 1., 0., 0., 0.],
           c_slots=[[.1 + i * .01, .04, .02, 1., 0., 0., 0.] for i in range(8)])


class IO:
    def __init__(self, count=0, failure=''):
        self.events, self.count, self.b, self.clear, self.failure = [], count, True, True, failure
    def guard(self):
        if self.failure == 'safety': raise RuntimeError('safety')
    def zone(self):
        return S(success=self.failure != 'vision', message='camera', b_has_object=self.b,
                 b_arm_clear=self.clear, c_object_count=self.count)
    def setup_scene(self): self.events.append('scene')
    def verify_layout(self, occupied):
        if self.failure == 'grid': raise RuntimeError('grid')
        self.events.append('camera_layout')
    def verify_slot(self, occupied, slot):
        if self.failure == 'wrong_slot': raise RuntimeError('wrong slot')
        self.events.append('camera_slot')
    def check_ik(self, p):
        self.events.append('ik')
        if self.failure == 'ik': raise RuntimeError('ik')
    def move_joints(self, p): self.events.append('home')
    def move_pose(self, p, linear=False):
        self.events.append(('linear' if linear else 'move', list(p)))
        if self.failure == 'path': raise RuntimeError('collision / incomplete path')
    def suction(self, op):
        self.events.append(op)
        if self.failure == op: raise RuntimeError(op)
        if op == 'grasp': self.b = False
        if op == 'release' and self.failure != 'count': self.count += 1
    def attach_disk(self): self.events.append('attach')
    def detach_disk(self): self.events.append('detach')
    def record_slot(self, p, slot): self.events.append(('record', slot))


def test_eight_in_order_and_ninth_no_motion():
    occupied = [False] * 8
    for slot in range(1, 9):
        io = IO(slot - 1)
        states = []
        w = Arm2Workflow(io, CFG, occupied, on_state=lambda s: states.append(s))
        w.run()
        occupied = w.occupied
        assert w.slot == slot and occupied == [True] * slot + [False] * (8 - slot)
        assert w.state == ('FULL' if slot == 8 else 'SUCCEEDED')
        assert not w.b_locked and w.holding == 'empty'
        assert io.events.index('verify_layout') < io.events.index('grasp')
        assert io.events.index('release') < io.events.index('verify_slot')
        assert len([e for e in io.events if isinstance(e, tuple) and e[0] == 'linear']) == 4
        assert all(s['b_locked'] for s in states if s['state'] in ('GRASP_B', 'HOME_WITH_OBJECT', 'VERIFY_B_EMPTY'))
        assert all(not s['occupied'][slot - 1] for s in states[:-1])
    io = IO(8)
    w = Arm2Workflow(io, CFG, occupied)
    w.run()
    assert w.state == 'FULL' and io.events == []


def test_smallest_empty_not_count_as_slot():
    io = IO(3)
    w = Arm2Workflow(io, CFG, [True, False, True, False, True, False, False, False])
    w.run()
    assert w.slot == 2


@pytest.mark.parametrize('failure,held,locked', [
    ('safety', 'empty', False), ('vision', 'empty', False), ('ik', 'empty', False),
    ('grid', 'empty', False), ('wrong_slot', 'empty', False),
    ('path', 'empty', True), ('grasp', 'unknown', True), ('release', 'unknown', False),
    ('count', 'empty', False), ('verify_slot', 'empty', False)])
def test_failures_never_commit_occupancy(failure, held, locked):
    io = IO(failure=failure)
    w = Arm2Workflow(io, CFG, [False] * 8)
    with pytest.raises((RuntimeError, TimeoutError)): w.run()
    assert w.state == 'FAULT' and not any(w.occupied)
    assert w.holding == held and w.b_locked == locked
    if failure in ('safety', 'vision', 'ik'):
        assert not any(isinstance(e, tuple) for e in io.events)


def test_b_blocked_no_motion_and_count_mismatch_no_motion():
    for io in (IO(), IO(1)):
        if io.count == 0: io.clear = False
        w = Arm2Workflow(io, CFG, [False] * 8)
        with pytest.raises(TimeoutError): w.run()
        assert io.events == []


def test_cancel_after_grasp_retains_b_and_unknown_tool():
    cancel, io = threading.Event(), IO()
    def confirm(op):
        if op == 'grasp':
            cancel.set()
            raise InterruptedError('cancel')
    io.suction = confirm
    w = Arm2Workflow(io, CFG, [False] * 8, cancel)
    with pytest.raises(InterruptedError): w.run()
    assert w.holding == 'unknown' and w.b_locked and not any(w.occupied)
    assert 'detach' not in io.events


def test_config_rejects_uncalibrated_nonfinite_duplicate_and_bad_orientation():
    c = dict(b_pick=CFG['b_pick'], c_poses=sum(CFG['c_slots'], []), home_joints=CFG['home_joints'],
             rows=2, columns=4, simulation=True, calibrated=False, standalone_arm1_secured=False,
             calibration_id='', safety_max_age=2., obstacle_boxes=[])
    assert len(validate_arm2(c)) == 8
    for patch in [dict(simulation=False), dict(b_pick=[float('nan')] * 7),
                  dict(c_poses=CFG['c_slots'][0] * 8), dict(b_pick=[.1,.1,.1,0.,0.,0.,0.])]:
        with pytest.raises(ValueError): validate_arm2({**c, **patch})


@pytest.mark.parametrize('occupancy', [[False]*7, [0]*8, ['false']*8, None])
def test_corrupt_journal_is_not_empty(occupancy):
    with pytest.raises(ValueError):
        validate_journal(dict(occupied=occupancy, holding='empty', b_locked=False))

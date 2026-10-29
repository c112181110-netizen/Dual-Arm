import threading
from types import SimpleNamespace as S
import pytest
from dual_arm_control.workflow import Arm1Workflow


CFG = dict(home_joints=[0.0, 0.0, 1.57, 1.57], home_settle=0.001,
           zone_wait_timeout=0.02, zone_poll_period=0.001,
           a_min=[0.08, 0.0, 0.04], a_max=[0.20, 0.10, 0.05],
           pick_z=0.045, height_tolerance=0.003, approach_height=0.03,
           b_place=[0.15, -0.04, 0.045])


class FakeIO:
    def __init__(self):
        self.events = []
        self.xyz = [0.15, 0.04, 0.045]
        self.b_occupied = False
        self.b_clear = True
        self.vision_success = True
        self.release_failure = False
        self.motion_failure = False
        self.verify_failure = False

    def setup_scene(self): self.events.append('scene')
    def clear_placed_disk(self): self.events.append('clear_old_b_model')
    def move_joints(self, q): self.events.append(('home', q))
    def move_pose(self, p, linear=False):
        self.events.append(('linear' if linear else 'move', p))
        if self.motion_failure: raise RuntimeError('planner failed')
    def attach_disk(self): self.events.append('attach')
    def detach_disk(self): self.events.append('detach')
    def record_placed_disk(self, p): self.events.append('placed_model')
    def zone(self):
        return S(success=self.vision_success, a_has_object=True, b_has_object=self.b_occupied,
                 b_arm_clear=self.b_clear, message='vision unavailable')
    def pick(self):
        self.events.append('pick_service')
        return S(success=True, x=self.xyz[0], y=self.xyz[1], z=self.xyz[2], message='')
    def suction(self, operation):
        self.events.append(operation)
        if operation == 'release':
            if self.release_failure: raise RuntimeError('release failed')
            self.b_occupied = not self.verify_failure


def test_success_requires_final_vision_confirmation_and_releases_lock():
    io, states = FakeIO(), []
    task = Arm1Workflow(io, CFG, on_state=lambda s: states.append(dict(s)))
    task.run()
    assert task.state == 'SUCCEEDED' and task.holding == 'empty' and not task.b_locked
    assert io.events.index('grasp') < io.events.index('attach') < io.events.index('release')
    assert all(s['b_locked'] for s in states if s['state'].startswith(('ARM1_STATE2', 'ARM1_STATE3')))
    assert [x[0] for x in io.events if isinstance(x, tuple)].count('linear') == 4


@pytest.mark.parametrize('xyz', [[float('nan'), 0.04, 0.045], [0.8, 0.04, 0.045], [0.15, 0.04, 0.049]])
def test_bad_vision_target_never_moves_to_pick_or_activates_suction(xyz):
    io = FakeIO(); io.xyz = xyz
    task = Arm1Workflow(io, CFG)
    with pytest.raises(ValueError): task.run()
    assert 'grasp' not in io.events
    assert not any(isinstance(e, tuple) and e[0] in ('move', 'linear') for e in io.events)


def test_planning_failure_never_advances_to_suction():
    io = FakeIO(); io.motion_failure = True
    task = Arm1Workflow(io, CFG)
    with pytest.raises(RuntimeError, match='planner'): task.run()
    assert task.state == 'FAULT' and 'grasp' not in io.events


def test_b_occupied_holds_object_without_entering_or_releasing():
    io = FakeIO(); io.b_occupied = True
    task = Arm1Workflow(io, CFG)
    with pytest.raises(TimeoutError): task.run()
    assert task.holding == 'held' and not task.b_locked
    assert 'release' not in io.events and 'clear_old_b_model' not in io.events


def test_release_failure_keeps_b_reserved_and_holding_unknown():
    io = FakeIO(); io.release_failure = True
    task = Arm1Workflow(io, CFG)
    with pytest.raises(RuntimeError, match='release failed'): task.run()
    assert task.b_locked and task.holding == 'unknown'
    assert 'detach' not in io.events


def test_unverified_placement_keeps_b_reserved():
    io = FakeIO(); io.verify_failure = True
    task = Arm1Workflow(io, CFG)
    with pytest.raises(TimeoutError): task.run()
    assert task.b_locked and task.state == 'FAULT'


def test_invalid_zone_response_is_not_treated_as_empty_b():
    io = FakeIO(); io.vision_success = False
    task = Arm1Workflow(io, CFG)
    with pytest.raises(RuntimeError, match='Invalid vision'): task.run()
    assert 'pick_service' not in io.events


def test_cancel_at_grasp_does_not_automatically_release_or_move():
    io, cancel = FakeIO(), threading.Event()
    def grasp(operation):
        io.events.append(operation)
        cancel.set()
        raise InterruptedError('operator canceled')
    io.suction = grasp
    task = Arm1Workflow(io, CFG, cancel)
    with pytest.raises(InterruptedError): task.run()
    assert task.holding == 'unknown' and 'release' not in io.events and 'attach' not in io.events

import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import rclpy
from dual_arm_interfaces.srv import ConfirmStep
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_interfaces.srv import GetPickTarget, GetZoneState

from .moveit_client import MoveItClient, wait_future
from .workflow import Arm1Workflow


DEFAULTS = {
    'simulation': True, 'calibrated': False, 'auto_confirm_demo': False,
    'planning_group': 'arm1', 'base_frame': 'arm1_base', 'tcp_link': 'arm1_suction_tcp',
    'joint_names': ['arm1_arm_shoulder_pan_joint', 'arm1_arm_shoulder_lift_joint',
                    'arm1_arm_elbow_flex_joint', 'arm1_arm_wrist_flex_joint'],
    'home_joints': [0.0, 0.0, 1.5708, 1.5708],
    'pick_z': 0.02, 'height_tolerance': 0.003, 'b_place': [0.15, -0.04, 0.02],
    'a_min': [0.08, 0.0, 0.017], 'a_max': [0.20, 0.10, 0.023], 'approach_height': 0.02,
    'tool_yaw_offset': 0.0, 'home_settle': 1.1, 'service_timeout': 4.0,
    'planning_timeout': 5.0, 'execution_margin': 20.0, 'joint_state_max_age': 1.0,
    'velocity_scale': 0.25, 'acceleration_scale': 0.25,
    'zone_wait_timeout': 30.0, 'zone_poll_period': 0.2, 'manual_timeout': 600.0,
    'target_rank': 0, 'peek_only': True,
    'pick_service': '/vision/arm1/get_pick_target', 'zone_service': '/vision/env/get_zone_state',
    'disk_radius': 0.01, 'disk_thickness': 0.002,
    'table_size': [0.6, 0.6, 0.02], 'table_center': [0.0, 0.0, -0.06],
    'journal_path': '~/.ros/dual_arm/arm1_journal.json',
}


def validate_config(c):
    if not c['simulation'] and not c['calibrated']:
        raise ValueError('Real operation requires a calibrated configuration')
    if c['auto_confirm_demo'] and not c['simulation']:
        raise ValueError('Automatic suction confirmation is simulation-only')
    for key in ('home_joints', 'b_place', 'a_min', 'a_max', 'table_size', 'table_center'):
        expected = 4 if key == 'home_joints' else 3
        if len(c[key]) != expected or not all(math.isfinite(v) for v in c[key]):
            raise ValueError(f'Invalid {key}')
    for key in ('home_settle', 'service_timeout', 'planning_timeout', 'execution_margin',
                'joint_state_max_age', 'zone_wait_timeout', 'zone_poll_period', 'manual_timeout',
                'height_tolerance', 'approach_height', 'disk_radius', 'disk_thickness'):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    for key in ('velocity_scale', 'acceleration_scale'):
        if not 0 < c[key] <= 1:
            raise ValueError(f'{key} must be in (0,1]')
    if not math.isfinite(c['pick_z']) or not math.isfinite(c['tool_yaw_offset']):
        raise ValueError('Invalid tool geometry')
    if any(lo >= hi for lo, hi in zip(c['a_min'], c['a_max'])):
        raise ValueError('A bounds must be increasing')
    if len(c['joint_names']) != 4 or len(set(c['joint_names'])) != 4:
        raise ValueError('Four distinct joint names required')


class Coordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        validate_config(self.cfg)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.confirm_event = threading.Event()
        self.confirm_result = None
        self.worker = None
        self.busy = False
        self.status = {'state': 'IDLE', 'cycle_id': '', 'holding': 'empty', 'b_locked': False,
                       'message': '', 'confirmation_token': '', 'confirmation_operation': ''}
        self.journal = Path(self.cfg['journal_path']).expanduser()
        self.journal.parent.mkdir(parents=True, exist_ok=True)
        if self.journal.exists():
            old = json.loads(self.journal.read_text())
            if old.get('state') not in ('IDLE', 'SUCCEEDED') or old.get('b_locked') or old.get('holding') != 'empty':
                self.status.update(old)
                self.status.update(state='FAULT', message='Previous cycle incomplete; inspect scene and recover explicitly',
                                   confirmation_token='', confirmation_operation='')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/arm1/status', qos)
        self.motion = MoveItClient(self, self.cfg, self.group, self.cancel)
        self.pick_client = self.create_client(GetPickTarget, self.cfg['pick_service'], callback_group=self.group)
        self.zone_client = self.create_client(GetZoneState, self.cfg['zone_service'], callback_group=self.group)
        self.create_service(Trigger, '/arm1/start', self.start, callback_group=self.group)
        self.create_service(Trigger, '/arm1/cancel', self.stop, callback_group=self.group)
        self.create_service(ConfirmStep, '/arm1/confirm', self.confirm, callback_group=self.group)
        self.create_service(ConfirmStep, '/arm1/recover', self.recover, callback_group=self.group)
        self.create_timer(1.0, self.publish)
        if self.status['state'] == 'FAULT':
            self.status['confirmation_token'] = uuid.uuid4().hex
            self.status['confirmation_operation'] = 'recovery'
        self.update({})

    def publish(self):
        with self.lock:
            self.publisher.publish(String(data=json.dumps({**self.status, 'busy': self.busy}, ensure_ascii=False)))

    def update(self, fields):
        with self.lock:
            changed = fields.get('state') != self.status['state'] if 'state' in fields else False
            self.status.update(fields)
            temp = self.journal.with_suffix('.tmp')
            with temp.open('w') as f:
                json.dump(self.status, f, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(temp, self.journal)
            self.publish()
            if changed:
                self.get_logger().info(self.status['state'])

    def start(self, request, response):
        with self.lock:
            if self.busy or self.status['state'] == 'FAULT':
                response.success, response.message = False, 'Busy or faulted; inspect /arm1/status'
                return response
            self.busy = True
            self.cancel.clear()
            self.update({'cycle_id': uuid.uuid4().hex, 'message': '', 'confirmation_token': '',
                         'confirmation_operation': '', 'state': 'STARTING'})
            self.worker = threading.Thread(target=self.run_cycle, daemon=True)
            self.worker.start()
            response.success = True
            response.message = f'Cycle accepted: {self.status["cycle_id"]}; completion is reported on /arm1/status'
        return response

    def run_cycle(self):
        try:
            Arm1Workflow(self, self.cfg, self.cancel, self.update).run()
            self.update({'message': 'A to B completed; B occupied and arm clear verified'})
        except Exception as e:
            self.motion.stop()
            self.update({'state': 'FAULT', 'message': str(e), 'confirmation_token': uuid.uuid4().hex,
                         'confirmation_operation': 'recovery'})
            self.get_logger().error(str(e))
        finally:
            with self.lock:
                self.busy = False
            self.publish()

    def stop(self, request, response):
        with self.lock:
            active = self.busy
            if active:
                self.cancel.set()
                self.confirm_event.set()
                self.motion.stop()
            response.success = active
            response.message = 'Cancellation requested; this is not a hardware emergency stop' if active else 'No active cycle'
        return response

    def vision_call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.cfg['service_timeout']):
            raise TimeoutError(f'Vision service unavailable: {client.srv_name}')
        return wait_future(client.call_async(request), self.cfg['service_timeout'], self.cancel)

    def zone(self):
        return self.vision_call(self.zone_client, GetZoneState.Request())

    def pick(self):
        return self.vision_call(self.pick_client, GetPickTarget.Request(
            target_rank=self.cfg['target_rank'], peek_only=self.cfg['peek_only']))

    def suction(self, operation):
        if self.cfg['auto_confirm_demo']:
            self.get_logger().warning(f'SIMULATION ONLY: artificial suction confirmation ({operation})')
            return
        with self.lock:
            self.confirm_event.clear()
            self.confirm_result = None
            self.update({'confirmation_token': uuid.uuid4().hex, 'confirmation_operation': operation,
                         'message': 'Operate suction externally, then confirm this exact token'})
        deadline = time.monotonic() + self.cfg['manual_timeout']
        try:
            while time.monotonic() < deadline:
                if self.cancel.is_set():
                    raise InterruptedError('Canceled while waiting for manual suction')
                if self.confirm_event.wait(0.05):
                    with self.lock:
                        result = self.confirm_result
                    if result is None or not result[0]:
                        raise RuntimeError('Operator reported suction failure' if result is None else result[1])
                    return
            raise TimeoutError('Manual suction confirmation timed out')
        finally:
            self.update({'confirmation_token': '', 'confirmation_operation': ''})

    def confirm(self, request, response):
        with self.lock:
            valid = (self.busy and not self.cancel.is_set() and request.token
                     and request.token == self.status['confirmation_token']
                     and self.status['confirmation_operation'] in ('grasp', 'release')
                     and self.confirm_result is None)
            if not valid:
                response.accepted, response.message = False, 'No matching pending manual step (stale or duplicate token)'
            else:
                self.confirm_result = (request.success, request.message or 'Operator reported failure')
                self.confirm_event.set()
                response.accepted, response.message = True, 'Confirmation recorded'
        return response

    def recover(self, request, response):
        # Explicit operator attestation: empty tool, clear B, restored Home_A.
        # This never commands a recovery motion or turns suction off.
        with self.lock:
            if self.busy or self.status['state'] != 'FAULT' or not request.success or not request.token or request.token != self.status['confirmation_token']:
                response.accepted, response.message = False, 'Requires idle FAULT and its current recovery token'
                return response
            self.busy = True
        try:
            self.cancel.clear()
            js = self.motion.state().joint_state
            if max(abs(a - b) for a, b in zip(js.position, self.cfg['home_joints'])) > 0.04:
                raise RuntimeError('Measured arm pose is not Home_A; recover physically first')
            zone = self.zone()
            if not zone.success or zone.b_has_object or not zone.b_arm_clear:
                raise RuntimeError('Vision does not confirm empty, clear B')
            self.motion.detach_disk()
            self.motion.clear_placed_disk()
            self.update({'state': 'IDLE', 'holding': 'empty', 'b_locked': False,
                         'message': 'Operator recovery acknowledged', 'confirmation_token': '', 'confirmation_operation': ''})
            response.accepted, response.message = True, 'Recovered; a new start is required'
        except Exception as e:
            response.accepted, response.message = False, str(e)
        finally:
            with self.lock:
                self.busy = False
        return response

    def __getattr__(self, name):
        if name in ('move_joints', 'move_pose', 'setup_scene', 'attach_disk', 'detach_disk',
                    'record_placed_disk', 'clear_placed_disk'):
            return getattr(self.motion, name)
        raise AttributeError(name)


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel.set()
        node.motion.stop()
        node.confirm_event.set()
        if node.worker:
            node.worker.join(timeout=5.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

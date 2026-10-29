"""Standalone arm2 B→C coordinator. Uses protocol-5 bridge and existing vision contract."""
import fcntl
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid

import rclpy
from dual_arm_interfaces.srv import ConfirmStep
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from vision_interfaces.srv import GetZoneState, GetGridState, VerifyPlace

from .coordinator import Coordinator, DEFAULTS, validate_config
from .moveit_client import MoveItClient
from .arm2_config import validate_arm2, validate_journal
from .arm2_workflow import Arm2Workflow

ARM2_DEFAULTS = {**DEFAULTS, 'planning_group': 'arm2', 'base_frame': 'arm2_base',
    'tcp_link': 'arm2_suction_tcp', 'controller_name': 'arm2_controller',
    'joint_names': [n.replace('arm1', 'arm2') for n in DEFAULTS['joint_names']],
    'b_pick': [float('nan')] * 7, 'c_poses': [float('nan')] * 56,
    'rows': 2, 'columns': 4, 'safety_max_age': 2.0,
    'standalone_arm1_secured': False, 'calibration_id': '', 'model_signature': '',
    'grid_verification': True, 'grid_position_tolerance': 0.01, 'require_size_order': False,
    'grid_service': '/vision/arm2/get_grid_state', 'verify_service': '/vision/arm2/verify_place',
    'obstacle_boxes': [0.0, 0.0, -0.2, 0.01, 0.01, 0.01],
    'journal_path': '~/.ros/dual_arm/arm2_journal.json'}


class Arm2Coordinator(Coordinator):
    def __init__(self):
        Node.__init__(self, 'arm2_coordinator')
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in ARM2_DEFAULTS.items()}
        validate_config(self.cfg)
        self.cfg['c_slots'] = validate_arm2(self.cfg)
        self.group, self.lock = ReentrantCallbackGroup(), threading.RLock()
        self.cancel, self.confirm_event = threading.Event(), threading.Event()
        self.confirm_result, self.worker, self.busy = None, None, False
        self.estop_latched = False
        self.safety = {}  # No messages means NOT safe; false does not reset a latched stop.
        self.journal = Path(self.cfg['journal_path']).expanduser()
        self.journal.parent.mkdir(parents=True, exist_ok=True)
        self.file_lock = self.journal.with_suffix('.lock').open('a')
        fcntl.flock(self.file_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        calibration = {k: v for k, v in self.cfg.items() if k not in
                       ('journal_path', 'auto_confirm_demo')}
        self.fingerprint = hashlib.sha256(json.dumps(calibration, sort_keys=True).encode()).hexdigest()
        self.status = dict(state='IDLE', cycle_id='', holding='empty', b_locked=False,
                           occupied=[False] * 8, slot=0, message='', confirmation_token='',
                           confirmation_operation='', config_fingerprint=self.fingerprint)
        if self.journal.exists():
            old = json.loads(self.journal.read_text())
            validate_journal(old)
            self.status.update(old)
            if (old.get('state') not in ('IDLE', 'SUCCEEDED', 'FULL') or old['b_locked']
                    or old['holding'] != 'empty' or old.get('config_fingerprint') != self.fingerprint):
                self.status.update(state='FAULT', message='Incomplete cycle or changed calibration; manual recovery required')
        self.status.update(confirmation_token='', confirmation_operation='')
        if self.status['state'] in ('FAULT', 'FULL'):
            self.status.update(confirmation_token=uuid.uuid4().hex, confirmation_operation='recovery')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/arm2/status', qos)
        self.handoff = self.create_publisher(String, '/handoff/b_state', qos)
        self.motion = MoveItClient(self, self.cfg, self.group, self.cancel)
        self.zone_client = self.create_client(GetZoneState, self.cfg['zone_service'], callback_group=self.group)
        self.grid_client = self.create_client(GetGridState, self.cfg['grid_service'], callback_group=self.group)
        self.verify_client = self.create_client(VerifyPlace, self.cfg['verify_service'], callback_group=self.group)
        self.read_scene = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=self.group)
        for name in ('start', 'b_ready'):
            self.create_service(Trigger, '/arm2/' + name, self.start, callback_group=self.group)
        self.create_service(Trigger, '/arm2/cancel', self.stop, callback_group=self.group)
        self.create_service(ConfirmStep, '/arm2/confirm', self.confirm, callback_group=self.group)
        self.create_service(ConfirmStep, '/arm2/recover', self.recover, callback_group=self.group)
        self.create_subscription(Bool, '/safety/estop', lambda m: self.safety_input('estop', m.data), 10, callback_group=self.group)
        self.create_subscription(Bool, '/handoff/arm1_clear', lambda m: self.safety_input('arm1_clear', m.data), 10, callback_group=self.group)
        self.create_timer(0.1, self.watchdog, callback_group=self.group)
        self.create_timer(0.5, self.publish)
        self.update({})

    def publish(self):
        super().publish()
        with self.lock:
            # Informational contract, NOT a distributed mutex. READY is a service input.
            signal = ('BUSY' if self.status['b_locked'] else
                      'B_EMPTY' if self.status['state'] in ('B_EMPTY', 'APPROACH_C', 'DESCEND_C',
                          'RELEASE_C', 'DETACH', 'LIFT_C', 'RETURN_HOME', 'VERIFY_C', 'SUCCEEDED', 'FULL')
                      else 'UNKNOWN')
            if self.status['state'] == 'FAULT':
                signal = 'BUSY' if self.status['b_locked'] else 'UNKNOWN'
            self.handoff.publish(String(data=json.dumps(dict(signal=signal, owner='arm2' if self.status['b_locked'] else '',
                b_occupied=True if self.status['state'] == 'B_RESERVED' else None,
                signals=([signal, 'B_OCCUPIED'] if self.status['state'] == 'B_RESERVED' else [signal]),
                busy=self.busy, cycle_id=self.status['cycle_id'], standalone=True))))

    def safety_input(self, key, value):
        with self.lock:
            self.safety[key] = (value, time.monotonic())
            if key == 'estop' and value:
                self.estop_latched = True
        self.watchdog()

    def guard(self):
        with self.lock:
            if self.estop_latched:
                raise RuntimeError('Emergency-stop input latched; inspect hardware before restarting')
            now = time.monotonic()
            for name, required in [('estop', False), ('arm1_clear', True)]:
                value, received = self.safety.get(name, (None, 0))
                if value is not required or now - received > self.cfg['safety_max_age']:
                    raise RuntimeError('Unsafe or stale safety input: ' + name)
            if self.cancel.is_set():
                raise InterruptedError('Canceled')

    def watchdog(self):
        if self.busy:
            try:
                self.guard()
            except Exception:
                self.cancel.set()
                self.confirm_event.set()
                self.motion.stop()

    def start(self, req, res):
        with self.lock:
            if self.busy or self.status['state'] == 'FAULT' or all(self.status['occupied']):
                res.success, res.message = False, 'Busy, faulted or C full; inspect /arm2/status'
                return res
            try:
                self.cancel.clear()
                self.guard()
            except Exception as e:
                res.success, res.message = False, str(e)
                return res
            self.busy = True
            self.update(dict(state='STARTING', cycle_id=uuid.uuid4().hex, message='',
                             confirmation_token='', confirmation_operation=''))
            self.worker = threading.Thread(target=self.run_cycle, daemon=True)
            self.worker.start()
            res.success, res.message = True, 'B_READY accepted once; completion is on /arm2/status'
            return res

    def run_cycle(self):
        try:
            Arm2Workflow(self, self.cfg, self.status['occupied'], self.cancel, self.update).run()
        except Exception as e:
            self.motion.stop()
            self.update(dict(state='FAULT', message=str(e), confirmation_token=uuid.uuid4().hex,
                             confirmation_operation='recovery'))
            self.get_logger().error(str(e))
        finally:
            with self.lock:
                self.busy = False
            self.publish()

    def confirm(self, req, res):
        with self.lock:
            if (not self.busy or self.cancel.is_set() or not req.token or
                    req.token != self.status['confirmation_token'] or self.confirm_result is not None or
                    self.status['confirmation_operation'] not in ('grasp', 'release', 'verify_layout', 'verify_slot')):
                res.accepted, res.message = False, 'No matching pending step (stale/duplicate token)'
            else:
                self.confirm_result = (req.success, req.message or 'Operator reported failure')
                self.confirm_event.set()
                res.accepted, res.message = True, 'Confirmation recorded'
            return res

    def setup_scene(self):
        self.guard()
        self.motion.setup_scene()
        scene = PlanningScene(is_diff=True)
        boxes = self.cfg['obstacle_boxes']
        for i in range(0, len(boxes), 6):
            obj = CollisionObject(id=f'arm2_obstacle_{i // 6}', operation=CollisionObject.ADD)
            obj.header.frame_id = self.cfg['base_frame']
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=boxes[i + 3:i + 6])]
            p = Pose()
            p.orientation.w = 1.0
            p.position.x, p.position.y, p.position.z = boxes[i:i + 3]
            obj.primitive_poses = [p]
            scene.world.collision_objects.append(obj)
        if scene.world.collision_objects:
            self.motion.apply_scene(scene)
        for i, taken in enumerate(self.status['occupied']):
            if taken:
                self.record_slot(self.cfg['c_slots'][i], i + 1)

    def record_slot(self, pose, slot):
        self.motion.record_placed_disk(pose[:3], f'c_slot_{slot}')

    def check_home(self):
        js = self.motion.state().joint_state
        if max(abs(a - b) for a, b in zip(js.position, self.cfg['home_joints'])) > 0.04:
            raise RuntimeError('Camera verification requires measured Home_C; restore physically first')

    def check_grid(self, occupied):
        grid = self.vision_call(self.grid_client, GetGridState.Request())
        if not grid.success:
            raise RuntimeError('Grid vision invalid: ' + grid.message)
        if list(grid.occupied) != list(occupied) or grid.all_full != all(occupied):
            raise RuntimeError('Camera slot occupancy disagrees with expected eight-slot map')
        for i, p in enumerate(self.cfg['c_slots']):
            xyz = (grid.place_x[i], grid.place_y[i], grid.place_z[i])
            if (not all(math.isfinite(v) for v in xyz) or
                    math.dist(xyz, p[:3]) > self.cfg['grid_position_tolerance']):
                raise RuntimeError(f'Camera C{i + 1} coordinates disagree with calibrated fixed pose')

    def verify_layout(self, occupied):
        self.guard()
        self.check_home()
        if self.cancel.wait(self.cfg['home_settle']):
            raise InterruptedError('Canceled during Home_C camera settling')
        if self.cfg['grid_verification']:
            self.check_grid(occupied)

    def verify_slot(self, occupied, slot):
        self.guard()
        self.check_home()
        if self.cfg['grid_verification']:
            result = self.vision_call(self.verify_client, VerifyPlace.Request(slot=slot))
            if not result.success or not result.placed:
                raise RuntimeError('Specified slot not verified: ' + result.message)
            if self.cfg['require_size_order'] and (not result.order_ok or result.wrong_slots):
                raise RuntimeError('Vision size order check failed')
            self.check_grid(occupied)

    def move_pose(self, *args, **kwargs):
        self.guard()
        return self.motion.move_pose(*args, **kwargs)

    def move_joints(self, *args, **kwargs):
        self.guard()
        return self.motion.move_joints(*args, **kwargs)

    def check_ik(self, pose):
        self.guard()
        return self.motion.check_ik(pose)

    def recover(self, req, res):
        # Recovery is deliberately a full physical clear-table operation. Never move here.
        with self.lock:
            if (self.busy or self.status['state'] not in ('FAULT', 'FULL') or not req.success
                    or not req.token or req.token != self.status['confirmation_token']):
                res.accepted, res.message = False, 'Requires idle FAULT/FULL and current recovery token'
                return res
            self.busy = True
        try:
            self.cancel.clear()
            self.guard()
            js = self.motion.state().joint_state
            if max(abs(a - b) for a, b in zip(js.position, self.cfg['home_joints'])) > 0.04:
                raise RuntimeError('Measured pose is not Home_C; restore physically first')
            z = self.zone()
            if not z.success or z.b_has_object or not z.b_arm_clear or z.c_object_count != 0:
                raise RuntimeError('Recovery requires empty B/C and clear B')
            self.motion.detach_disk()
            scene = PlanningScene(is_diff=True)
            # Also remove a stale attached object when the coordinator restarted
            # while an external MoveIt process retained its planning scene.
            scene.robot_state.is_diff = True
            current_scene = self.motion.service(self.read_scene, GetPlanningScene.Request(
                components=PlanningSceneComponents(components=(PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                                                PlanningSceneComponents.WORLD_OBJECT_NAMES)))).scene
            scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                link_name=obj.link_name, object=CollisionObject(id='carried_disk', operation=CollisionObject.REMOVE))
                for obj in current_scene.robot_state.attached_collision_objects if obj.object.id == 'carried_disk']
            owned = {f'c_slot_{i}' for i in range(1, 9)} | {'carried_disk'}
            scene.world.collision_objects = [CollisionObject(id=obj.id, operation=CollisionObject.REMOVE)
                                             for obj in current_scene.world.collision_objects if obj.id in owned]
            if scene.world.collision_objects or scene.robot_state.attached_collision_objects:
                self.motion.apply_scene(scene)
            if self.cfg['grid_verification']:
                self.check_grid([False] * 8)
            self.update(dict(state='IDLE', holding='empty', b_locked=False, occupied=[False] * 8, slot=0,
                             confirmation_token='', confirmation_operation='', message='Operator cleared B/C and tool; new B_READY required',
                             config_fingerprint=self.fingerprint))
            res.accepted, res.message = True, 'Recovered without motion'
        except Exception as e:
            res.accepted, res.message = False, str(e)
        finally:
            with self.lock:
                self.busy = False
        return res

    def update(self, fields):
        if fields.get('state') == 'FULL':
            fields = {**fields, 'confirmation_token': uuid.uuid4().hex, 'confirmation_operation': 'recovery'}
        super().update(fields)


def main(args=None):
    rclpy.init(args=args)
    node = Arm2Coordinator()
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
            node.worker.join(timeout=5)
        executor.shutdown()
        node.file_lock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

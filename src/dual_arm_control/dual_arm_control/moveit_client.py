"""Use real MoveIt services/actions; no local substitute for IK or planning."""
import math
import threading
import time
from copy import deepcopy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                            JointConstraint, PlanningScene, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


def duration(value):
    from builtin_interfaces.msg import Duration
    return Duration(sec=int(value), nanosec=int((value % 1) * 1e9))


def wait_future(future, timeout, cancel=None):
    deadline = time.monotonic() + timeout
    while not future.done():
        if cancel is not None and cancel.is_set():
            raise InterruptedError('Task canceled')
        if time.monotonic() >= deadline:
            raise TimeoutError('ROS request timed out')
        time.sleep(0.01)
    return future.result()


class MoveItClient:
    def __init__(self, node, config, callback_group, cancel):
        self.node, self.cfg, self.cancel = node, config, cancel
        self.state_lock = threading.Lock()
        self.latest = None
        self.received = 0.0
        self.active_goal = None
        self.goal_lock = threading.Lock()
        self.attached_disk_recorded = False
        self.placed_disk_recorded = False
        self.sub = node.create_subscription(JointState, '/joint_states', self.on_state, 10, callback_group=callback_group)
        self.planner = ActionClient(node, MoveGroup, '/move_action', callback_group=callback_group)
        self.executor = ActionClient(node, ExecuteTrajectory, '/execute_trajectory', callback_group=callback_group)
        self.ik = node.create_client(GetPositionIK, '/compute_ik', callback_group=callback_group)
        self.cartesian = node.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=callback_group)
        self.scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=callback_group)

    def on_state(self, msg):
        names = self.cfg['joint_names']
        if all(n in msg.name for n in names) and len(msg.position) == len(msg.name):
            values = [msg.position[msg.name.index(n)] for n in names]
            if not all(math.isfinite(q) for q in values):
                return
            with self.state_lock:
                self.latest = JointState(name=names, position=values, header=msg.header)
                self.received = time.monotonic()

    def state(self):
        deadline = time.monotonic() + self.cfg['service_timeout']
        while time.monotonic() < deadline:
            if self.cancel.is_set():
                raise InterruptedError('Task canceled')
            with self.state_lock:
                if self.latest is not None and time.monotonic() - self.received <= self.cfg['joint_state_max_age']:
                    return RobotState(joint_state=deepcopy(self.latest), is_diff=True)
            time.sleep(0.02)
        raise TimeoutError('No fresh measured /joint_states')

    def service(self, client, request, timeout=None):
        timeout = timeout or self.cfg['service_timeout']
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'Service unavailable: {client.srv_name}')
        return wait_future(client.call_async(request), timeout, self.cancel)

    def action(self, client, goal, timeout):
        if self.cancel.is_set():
            raise InterruptedError('Task canceled')
        if not client.wait_for_server(timeout_sec=self.cfg['service_timeout']):
            raise TimeoutError('MoveIt action server unavailable')
        sent = client.send_goal_async(goal)
        try:
            handle = wait_future(sent, self.cfg['service_timeout'], self.cancel)
        except Exception:
            # A late acceptance must not result in untracked execution.
            def cancel_late(f):
                try:
                    h = f.result()
                    if h and h.accepted:
                        h.cancel_goal_async()
                except Exception:
                    pass
            sent.add_done_callback(cancel_late)
            raise
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected the action goal')
        with self.goal_lock:
            self.active_goal = handle
        try:
            result = wait_future(handle.get_result_async(), timeout, self.cancel)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != 1:
                raise RuntimeError(f'MoveIt failed: status={result.status}, code={result.result.error_code.val}')
            return result.result
        except Exception:
            handle.cancel_goal_async()
            raise
        finally:
            with self.goal_lock:
                self.active_goal = None

    def stop(self):
        with self.goal_lock:
            if self.active_goal is not None:
                self.active_goal.cancel_goal_async()

    def pose(self, xyz):
        # Pincher has four joints. For this model, downward suction fixes roll
        # and pitch while yaw follows shoulder pan; disk rotation is irrelevant.
        yaw = math.atan2(xyz[1], xyz[0]) + self.cfg['tool_yaw_offset']
        p = Pose()
        p.position.x, p.position.y, p.position.z = map(float, xyz[:3])
        p.orientation = (Quaternion(**dict(zip(('x', 'y', 'z', 'w'), map(float, xyz[3:]))))
                         if len(xyz) == 7 else
                         Quaternion(x=math.cos(yaw / 2), y=math.sin(yaw / 2), z=0.0, w=0.0))
        return p

    def move_joints(self, positions):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.cfg['planning_group']
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.cfg['planning_timeout']
        req.max_velocity_scaling_factor = self.cfg['velocity_scale']
        req.max_acceleration_scaling_factor = self.cfg['acceleration_scale']
        req.start_state = self.state()
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(q), tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0)
            for n, q in zip(self.cfg['joint_names'], positions)])]
        goal.planning_options.plan_only = True
        result = self.action(self.planner, goal, self.cfg['planning_timeout'] + 10.0)
        self.execute(result.planned_trajectory)

    def move_pose(self, xyz, linear=False):
        if linear:
            request = GetCartesianPath.Request()
            request.header.frame_id = self.cfg['base_frame']
            request.start_state = self.state()
            request.group_name, request.link_name = self.cfg['planning_group'], self.cfg['tcp_link']
            request.waypoints = [self.pose(xyz)]
            request.max_step = 0.002
            request.revolute_jump_threshold = 0.15
            request.avoid_collisions = True
            request.max_velocity_scaling_factor = self.cfg['velocity_scale']
            request.max_acceleration_scaling_factor = self.cfg['acceleration_scale']
            result = self.service(self.cartesian, request, self.cfg['planning_timeout'] + 5.0)
            if result.error_code.val != 1 or result.fraction < 0.999:
                raise RuntimeError(f'Incomplete Cartesian path: fraction={result.fraction:.3f}, code={result.error_code.val}')
            self.execute(result.solution)
            return
        self.move_joints(self.check_ik(xyz))

    def check_ik(self, xyz):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.cfg['planning_group']
        req.ik_request.ik_link_name = self.cfg['tcp_link']
        req.ik_request.robot_state = self.state()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = PoseStamped(pose=self.pose(xyz))
        req.ik_request.pose_stamped.header.frame_id = self.cfg['base_frame']
        req.ik_request.timeout = duration(1.0)
        result = self.service(self.ik, req)
        if result.error_code.val != 1:
            raise RuntimeError(f'No collision-free IK for {xyz}: code={result.error_code.val}')
        js = result.solution.joint_state
        return [js.position[js.name.index(n)] for n in self.cfg['joint_names']]

    def execute(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError('MoveIt returned an empty trajectory')
        last = points[-1].time_from_start
        timeout = last.sec + last.nanosec * 1e-9 + self.cfg['execution_margin']
        goal = ExecuteTrajectory.Goal(trajectory=trajectory, controller_names=[self.cfg.get('controller_name', 'arm1_controller')])
        self.action(self.executor, goal, timeout)

    def apply_scene(self, scene):
        response = self.service(self.scene, ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise RuntimeError('MoveIt rejected planning scene update')

    def setup_scene(self):
        scene = PlanningScene(is_diff=True)
        box = CollisionObject(id='work_surface', operation=CollisionObject.ADD)
        box.header.frame_id = self.cfg['base_frame']
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=self.cfg['table_size'])]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = self.cfg['table_center']
        box.primitive_poses = [pose]
        scene.world.collision_objects = [box]
        self.apply_scene(scene)

    def attach_disk(self):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        obj = CollisionObject(id='carried_disk', operation=CollisionObject.ADD)
        obj.header.frame_id = self.cfg['tcp_link']
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                       dimensions=[self.cfg['disk_thickness'], self.cfg['disk_radius']])]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.z = self.cfg['disk_thickness'] / 2.0
        obj.primitive_poses = [pose]
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name=self.cfg['tcp_link'], object=obj, touch_links=[self.cfg['tcp_link']])]
        self.apply_scene(scene)
        self.attached_disk_recorded = True

    def detach_disk(self):
        if not self.attached_disk_recorded:
            return
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name=self.cfg['tcp_link'], object=CollisionObject(id='carried_disk', operation=CollisionObject.REMOVE))]
        scene.world.collision_objects = [CollisionObject(id='carried_disk', operation=CollisionObject.REMOVE)]
        self.apply_scene(scene)
        self.attached_disk_recorded = False

    def record_placed_disk(self, xyz, object_id='placed_disk_b'):
        scene = PlanningScene(is_diff=True)
        obj = CollisionObject(id=object_id, operation=CollisionObject.ADD)
        obj.header.frame_id = self.cfg['base_frame']
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                       dimensions=[self.cfg['disk_thickness'], self.cfg['disk_radius']])]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.position.z -= self.cfg['disk_thickness'] / 2.0
        obj.primitive_poses = [pose]
        scene.world.collision_objects = [obj]
        self.apply_scene(scene)
        self.placed_disk_recorded = True

    def clear_placed_disk(self):
        if not self.placed_disk_recorded:
            return
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [CollisionObject(id='placed_disk_b', operation=CollisionObject.REMOVE)]
        self.apply_scene(scene)
        self.placed_disk_recorded = False

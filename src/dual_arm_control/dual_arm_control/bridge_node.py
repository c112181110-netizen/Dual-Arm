import threading
import time
import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .cm530 import Calibration, MockCM530, SerialCM530
from .trajectory import sample, seconds, validate_trajectory


class CM530Bridge(Node):
    def __init__(self):
        super().__init__('cm530_bridge')
        self.group = ReentrantCallbackGroup()
        self.estop = threading.Event()
        self.safety_inputs = {}
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.mock = param('mock_hardware', True)
        self.require_safety = param('require_safety_inputs', False)
        self.safety_max_age = param('safety_max_age', 2.0)
        self.period = param('command_period', 0.05)
        self.arrival_timeout = param('arrival_timeout', 10.0)
        self.tolerance = param('position_tolerance', 0.025)
        self.start_tolerance = param('start_tolerance', 0.04)
        self.stable_count = param('stable_count', 3)
        self.max_lag = param('max_schedule_lag', 0.25)
        self.time_scale = param('mock_time_scale', 1.0) if self.mock else 1.0
        timing = (self.period, self.arrival_timeout, self.tolerance, self.time_scale,
                  self.max_lag, self.start_tolerance, self.safety_max_age)
        if any(not math.isfinite(v) or v <= 0 for v in timing) or self.stable_count < 1:
            raise ValueError('Invalid controller timing/tolerance configuration')
        self.arms = param('arms', ['arm1'])
        if not self.arms or len(set(self.arms)) != len(self.arms) or any(a not in ('arm1', 'arm2') for a in self.arms):
            raise ValueError('arms must be a unique subset of arm1, arm2')
        self.names, self.calibration, initial = {}, {}, {}
        for arm in self.arms:
            stem = arm + '.'
            self.names[arm] = param(stem + 'joint_names', [arm + '_' + j for j in (
                'arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint')])
            if len(self.names[arm]) != 4 or len(set(self.names[arm])) != 4:
                raise ValueError('Four distinct joint names are required')
            self.calibration[arm] = Calibration(
                param(stem + 'zero_raw', [512, 512, 512, 512]),
                param(stem + 'direction', [1, 1, 1, 1]),
                param(stem + 'lower_rad', [-2.61799, -2.09439, -2.426, -1.7104]),
                param(stem + 'upper_rad', [2.61799, 2.09439, 2.426, 1.7976]),
                param(stem + 'raw_min', [0, 0, 0, 0]),
                param(stem + 'raw_max', [1023, 1023, 1023, 1023]))
            initial_rad = param(stem + 'initial_rad', [0.0, 0.0, 1.5708, 1.5708])
            if self.mock:
                initial[arm] = self.calibration[arm].to_raw(initial_rad)
        calibrated = param('hardware_calibrated', False)
        port = param('serial_port', '')
        timeout = param('command_timeout', 2.0)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('command_timeout must be positive and finite')
        if not self.mock and (not calibrated or not port):
            raise ValueError('Real hardware requires hardware_calibrated=true and an explicit serial_port')
        self.hardware = MockCM530(initial) if self.mock else SerialCM530(port, timeout)
        if not self.mock:
            for arm in self.arms:
                self.get_logger().info(f'{arm} firmware HOME (raw, not commanded): {self.hardware.transact("GET_HOME," + arm)}')
        self.enabled = {a: self.mock for a in self.arms}
        self.busy = False
        self.busy_lock = threading.Lock()
        self.fault = ''
        self.create_subscription(Bool, '/safety/estop', self.on_estop, 10, callback_group=self.group)
        self.create_subscription(Bool, '/handoff/arm1_clear',
            lambda m: self.safety_inputs.update(arm1_clear=(m.data, time.monotonic())), 10, callback_group=self.group)
        self.states = self.create_publisher(JointState, '/joint_states', 10)
        self.health = self.create_publisher(String, '/cm530/status', 10)
        self.servers = []
        for arm in self.arms:
            self.create_service(Trigger, f'/cm530/{arm}/enable',
                                lambda req, res, a=arm: self.enable(a, res), callback_group=self.group)
            self.servers.append(ActionServer(
                self, FollowJointTrajectory, f'/{arm}_controller/follow_joint_trajectory',
                execute_callback=lambda goal, a=arm: self.execute(a, goal),
                goal_callback=lambda req, a=arm: self.accept(a, req),
                cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=self.group))
        self.create_timer(0.1, self.poll, callback_group=self.group)
        self.get_logger().info('CM530 mock actuator ready' if self.mock else 'CM530 connected; torque must be explicitly enabled')

    def on_estop(self, msg):
        self.safety_inputs['estop'] = (msg.data, time.monotonic())
        if not msg.data:
            return
        self.estop.set()  # Latched until an inspected restart; false is not a reset.
        self.fault = 'Emergency-stop input latched'
        with self.busy_lock:
            if self.busy:
                return  # Active executor observes the latch and requests HOLD.
            self.busy = True
        try:
            if self.hardware.healthy:
                for arm in self.arms:
                    self.hardware.hold(arm)
        except Exception as e:
            self.fault += ': HOLD failed: ' + str(e)
        finally:
            with self.busy_lock:
                self.busy = False

    def check_safety(self):
        if self.estop.is_set():
            raise InterruptedError('Emergency-stop input latched')
        if self.require_safety:
            for name, required in [('estop', False), ('arm1_clear', True)]:
                value, stamp = self.safety_inputs.get(name, (None, 0))
                if value is not required or time.monotonic() - stamp > self.safety_max_age:
                    raise RuntimeError('Unsafe or stale safety input: ' + name)

    def enable(self, arm, response):
        with self.busy_lock:
            if self.busy or self.fault:
                response.success, response.message = False, self.fault or 'Controller busy'
                return response
            self.busy = True
        try:
            self.check_safety()
            self.hardware.enable(arm)
            self.check_safety()
            self.read_publish(arm)
            self.enabled[arm] = True
            response.success, response.message = True, 'HOLD followed by TORQUE enabled; no HOME command sent'
        except Exception as e:
            self.fault = str(e)
            response.success, response.message = False, str(e)
        finally:
            with self.busy_lock:
                self.busy = False
        return response

    def read_publish(self, arm):
        q = self.calibration[arm].to_rad(self.hardware.read(arm))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = self.names[arm], q
        self.states.publish(msg)
        return q

    def poll(self):
        with self.busy_lock:
            if self.busy or self.fault:
                self.health.publish(String(data=self.fault or 'executing'))
                return
        try:
            for arm in self.arms:
                self.read_publish(arm)
            self.health.publish(String(data='mock_ready' if self.mock else 'ready'))
        except Exception as e:
            self.fault = str(e)
            self.get_logger().error(self.fault)

    def accept(self, arm, request):
        try:
            self.check_safety()
            validate_trajectory(request.trajectory, self.names[arm], self.calibration[arm])
            if request.multi_dof_trajectory.points or request.component_path_tolerance or request.component_goal_tolerance:
                raise ValueError('Multi-DOF trajectories/tolerances are not supported')
            if seconds(request.trajectory.header.stamp) != 0:
                raise ValueError('Only relative trajectories (zero header stamp) are supported')
            for t in list(request.path_tolerance) + list(request.goal_tolerance):
                if not all(math.isfinite(v) for v in (t.position, t.velocity, t.acceleration)):
                    raise ValueError('Non-finite tolerance')
                if t.name not in self.names[arm] or t.velocity not in (0.0, -1.0) or t.acceleration not in (0.0, -1.0):
                    raise ValueError('Only named position tolerances are supported')
                if t.position < 0 and t.position != -1:
                    raise ValueError('Invalid position tolerance')
            with self.busy_lock:
                if self.busy or self.fault or not self.enabled[arm]:
                    raise ValueError(self.fault or 'Controller busy or torque not enabled')
                self.busy = True
            return GoalResponse.ACCEPT
        except Exception as e:
            self.get_logger().warning(f'Rejecting trajectory: {e}')
            return GoalResponse.REJECT

    def execute(self, arm, goal):
        result = FollowJointTrajectory.Result()
        try:
            request = goal.request
            times, points = validate_trajectory(request.trajectory, self.names[arm], self.calibration[arm])
            current = self.read_publish(arm)
            if times[0] == 0:
                if max(abs(a - b) for a, b in zip(current, points[0])) > self.start_tolerance:
                    raise RuntimeError('Trajectory start differs from measured pose')
            else:
                times.insert(0, 0.0)
                points.insert(0, current)
            started, next_tick = time.monotonic(), 0.0
            path_tol = {t.name: t.position for t in request.path_tolerance if t.position > 0}
            while True:
                self.check_safety()
                if goal.is_cancel_requested or not rclpy.ok():
                    raise InterruptedError('Trajectory canceled')
                now = (time.monotonic() - started) / self.time_scale
                if now < next_tick:
                    time.sleep(min(0.01, (next_tick - now) * self.time_scale))
                    continue
                if (now - next_tick) * self.time_scale > self.max_lag:
                    raise RuntimeError('Serial scheduling fell behind; trajectory aborted')
                desired = sample(times, points, min(next_tick, times[-1]))
                self.hardware.write(arm, self.calibration[arm].to_raw(desired))
                actual = self.read_publish(arm)
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = self.names[arm]
                feedback.desired = JointTrajectoryPoint(positions=desired)
                feedback.actual = JointTrajectoryPoint(positions=actual)
                feedback.error = JointTrajectoryPoint(positions=[a - b for a, b in zip(desired, actual)])
                goal.publish_feedback(feedback)
                for i, name in enumerate(self.names[arm]):
                    if name in path_tol and abs(desired[i] - actual[i]) > path_tol[name]:
                        result.error_code = result.PATH_TOLERANCE_VIOLATED
                        raise RuntimeError(f'Path tolerance violated: {name}')
                if next_tick >= times[-1]:
                    break
                next_tick = min(next_tick + self.period, times[-1])
            tolerance = {n: self.tolerance for n in self.names[arm]}
            for t in request.goal_tolerance:
                if t.position > 0:
                    tolerance[t.name] = t.position
                elif t.position == -1:
                    tolerance[t.name] = float('inf')
            extra = seconds(request.goal_time_tolerance)
            deadline = time.monotonic() + (extra if extra > 0 else self.arrival_timeout)
            stable = 0
            while time.monotonic() < deadline:
                self.check_safety()
                if goal.is_cancel_requested or not rclpy.ok():
                    raise InterruptedError('Canceled during arrival verification')
                actual = self.read_publish(arm)
                within = all(abs(a - b) <= tolerance[n] for n, a, b in zip(self.names[arm], points[-1], actual))
                stable = stable + 1 if within else 0
                if stable >= self.stable_count:
                    goal.succeed()
                    result.error_code = result.SUCCESSFUL
                    result.error_string = 'Final pose verified by fresh READ samples'
                    return result
                time.sleep(self.period * self.time_scale)
            result.error_code = result.GOAL_TOLERANCE_VIOLATED
            raise RuntimeError('Final joint positions did not settle before deadline')
        except Exception as e:
            # Never send HOLD onto a desynchronized serial connection.
            if self.hardware.healthy:
                try:
                    self.hardware.hold(arm)
                except Exception as hold_error:
                    self.fault = f'HOLD failed: {hold_error}'
            else:
                self.fault = str(e)
            if isinstance(e, InterruptedError) and goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
            if result.error_code == result.SUCCESSFUL:
                result.error_code = result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(e)
            self.get_logger().error(str(e))
            return result
        finally:
            with self.busy_lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = CM530Bridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.hardware.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

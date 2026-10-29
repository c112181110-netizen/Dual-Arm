"""SIMULATION ONLY: synthetic B supply, C counts and safety heartbeat."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from vision_interfaces.srv import GetZoneState, GetGridState, VerifyPlace


class Arm2MockVision(Node):
    def __init__(self):
        super().__init__('arm2_mock_vision')
        self.b, self.clear, self.count = True, True, 0
        self.block_b = self.declare_parameter('block_b', False).value
        self.fail_count = self.declare_parameter('fail_count', False).value
        self.fail_grid = self.declare_parameter('fail_grid', False).value
        self.wrong_slot = self.declare_parameter('wrong_slot', False).value
        self.grid_mismatch = self.declare_parameter('grid_mismatch', False).value
        self.c_poses = self.declare_parameter('c_poses', [0.0] * 56).value
        self.occupied = [False] * 8
        self.previous = ''
        self.freeze_on_grasp = self.declare_parameter('freeze_safety_on_grasp', False).value
        self.freeze_safety = False
        self.create_service(GetZoneState, '/vision/env/get_zone_state', self.zone)
        self.create_service(GetGridState, '/vision/arm2/get_grid_state', self.grid)
        self.create_service(VerifyPlace, '/vision/arm2/verify_place', self.verify)
        self.create_service(Trigger, '/arm2_mock/clear_table', self.clear_table)
        self.create_subscription(String, '/arm2/status', self.status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.clear_pub = self.create_publisher(Bool, '/handoff/arm1_clear', 10)
        self.stop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.create_timer(0.1, self.heartbeat)
        self.get_logger().warning('SIMULATED scene and safety inputs; NEVER run with hardware')

    def heartbeat(self):
        if self.freeze_safety:
            return
        self.clear_pub.publish(Bool(data=True))
        self.stop_pub.publish(Bool(data=False))

    def zone(self, req, res):
        res.success, res.message = True, 'synthetic scene, not camera evidence'
        res.a_has_object = False
        res.b_has_object, res.b_arm_clear = self.b, self.clear and not self.block_b
        res.c_object_count = self.count
        return res

    def status(self, msg):
        value = json.loads(msg.data)
        state = value['state']
        if state == self.previous:
            return
        self.previous = state
        if state == 'CHECK_B_READY':
            self.b = True
            self.count = sum(value['occupied'])
            self.occupied = list(value['occupied'])
        if state == 'B_RESERVED':
            self.clear = False
        if state == 'GRASP_B' and self.freeze_on_grasp:
            self.freeze_safety = True
        if state == 'ATTACH':
            self.b = False
        if state == 'VERIFY_B_EMPTY':
            self.clear = True
        if state == 'DETACH' and not self.fail_count:
            self.count += 1
            slot = value['slot'] - 1
            self.occupied[(slot + 1) % 8 if self.wrong_slot else slot] = True

    def grid(self, req, res):
        res.success, res.message = not self.fail_grid, 'synthetic arm2 camera'
        res.occupied, res.all_full = self.occupied, all(self.occupied)
        res.place_x = [self.c_poses[i * 7] + (1.0 if self.grid_mismatch else 0.0) for i in range(8)]
        res.place_y = [self.c_poses[i * 7 + 1] for i in range(8)]
        res.place_z = [self.c_poses[i * 7 + 2] for i in range(8)]
        return res

    def verify(self, req, res):
        res.success = not self.fail_grid and 1 <= req.slot <= 8
        res.placed = bool(res.success and self.occupied[req.slot - 1])
        res.order_ok, res.wrong_slots = True, []
        res.message = 'synthetic slot observation'
        return res

    def clear_table(self, req, res):
        self.b, self.clear, self.count = False, True, 0
        self.occupied = [False] * 8
        res.success, res.message = True, 'Synthetic B/C cleared'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = Arm2MockVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

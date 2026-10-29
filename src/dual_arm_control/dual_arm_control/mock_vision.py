"""Contract simulator only. Does not implement or replace the vision team's code."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_interfaces.srv import GetPickTarget, GetZoneState


class MockVision(Node):
    def __init__(self):
        super().__init__('mock_vision')
        self.xyz = self.declare_parameter('pick_xyz', [0.15, 0.04, 0.02]).value
        self.block_b = self.declare_parameter('block_b', False).value
        self.a, self.b, self.clear = True, False, True
        self.create_service(GetPickTarget, '/vision/arm1/get_pick_target', self.pick)
        self.create_service(GetZoneState, '/vision/env/get_zone_state', self.zone)
        self.create_service(Trigger, '/mock_vision/reset', self.reset)
        self.create_subscription(String, '/arm1/status', self.status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().warning('MOCK vision services: synthetic coordinates and scene state')

    def pick(self, req, res):
        res.success = self.a
        res.message = 'synthetic target' if self.a else 'no object in A'
        res.rank = 1
        res.x, res.y, res.z = self.xyz
        return res

    def zone(self, req, res):
        res.success, res.message = True, 'synthetic scene'
        res.a_has_object, res.b_has_object = self.a, self.b or self.block_b
        res.b_arm_clear = self.clear
        res.c_object_count = 0
        return res

    def status(self, msg):
        state = json.loads(msg.data)['state']
        if state == 'ARM1_STATE1_ATTACH':
            self.a = False
        if state == 'ARM1_STATE2_DETACH':
            self.b = True
        if state in ('ARM1_STATE2_APPROACH_B', 'ARM1_STATE2_DESCEND_B', 'ARM1_STATE2_WAIT_SUCTION_OFF',
                     'ARM1_STATE2_DETACH', 'ARM1_STATE3_LIFT_B', 'ARM1_STATE3_HOME'):
            self.clear = False
        if state in ('ARM1_STATE3_VERIFY_B', 'SUCCEEDED', 'IDLE'):
            self.clear = True

    def reset(self, req, res):
        self.a, self.b, self.clear = True, False, True
        res.success, res.message = True, 'Synthetic scene reset'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = MockVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

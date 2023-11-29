#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class ArmControlNode(Node):
    def __init__(self):
        super().__init__("arm_control_node")

        self._cmd_vel_pub = self.create_publisher(
            msg_type=JointState,
            topic="/joint_states",
            qos_profile=10
        )

        self._arm_angle = 0.0

        self._timer = self.create_timer(0.1, self._pub_timer_callback)

    def _pub_timer_callback(self):
        self.get_logger().info("Angle: %.4f" % self._arm_angle)

        msg = JointState(
            header=Header(stamp=self.get_clock().now().to_msg()),
            name=['arm_joint', 'slider_joint'],
            position=[self._arm_angle, 0.0],
            velocity=[],
            effort=[]
        )

        self._arm_angle += 0.05
        if self._arm_angle >= math.pi / 2.0:
            self._arm_angle = 0.0

        self._cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ArmControlNode()

    rclpy.spin(node)

    # Cleanup
    rclpy.shutdown()


if __name__ == "__main__":
    main()

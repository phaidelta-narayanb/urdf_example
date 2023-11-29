#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        self.get_logger().info("This will publish a new trajectory to Gazebo every often.")

        self._cmd_vel_pub = self.create_publisher(
            msg_type=JointTrajectory,
            topic="/set_joint_trajectory",
            qos_profile=10
        )

        self._timer = self.create_timer(2, self._pub_timer_callback)


    def _pub_timer_callback(self):
        self._cmd_vel_pub.publish(JointTrajectory(
            header=Header(
                frame_id="world"
            ),
            joint_names=["slider_joint", "arm_joint"],
            points=[
                JointTrajectoryPoint(positions=(1.0, 1.0))
            ]
        ))


def main(args=None):
    rclpy.init(args=args)

    node = ArmControlNode()

    rclpy.spin(node)

    # Cleanup
    rclpy.shutdown()


if __name__ == "__main__":
    main()

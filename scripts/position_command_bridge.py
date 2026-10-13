#!/usr/bin/env python3
"""Optional EGO-Planner quadrotor_msgs -> px4ctrl2 trajectory adapter."""
import rclpy
from rclpy.node import Node
from quadrotor_msgs.msg import PositionCommand as PlannerCommand
from px4ctrl2.msg import PositionCommand


class Bridge(Node):
    def __init__(self):
        super().__init__('px4ctrl2_position_command_bridge')
        source = self.declare_parameter('input_topic', '/position_cmd').value
        target = self.declare_parameter('output_topic', '/px4ctrl2/command').value
        self.publisher = self.create_publisher(PositionCommand, target, 10)
        self.subscription = self.create_subscription(PlannerCommand, source, self.convert, 10)

    def convert(self, source):
        # The current EGO message lacks jerk; original LinearControl does not use it.
        if source.trajectory_flag != PlannerCommand.TRAJECTORY_STATUS_READY:
            return
        out = PositionCommand()
        out.header = source.header
        out.position = source.position
        out.velocity = source.velocity
        out.acceleration = source.acceleration
        out.yaw = source.yaw
        out.yaw_dot = source.yaw_dot
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""RViz 2D goal -> smooth px4ctrl2 position commands in the VIO global frame."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from px4ctrl2.msg import PositionCommand
from goal_trajectory import GoalTrajectory


def yaw(q):
    values = np.array([q.x, q.y, q.z, q.w])
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid goal orientation')
    x, y, z, w = values/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class GoalNode(Node):
    def __init__(self):
        super().__init__('rviz_goal')
        self.speed = self.declare_parameter('max_speed', 0.4).value
        self.accel = self.declare_parameter('max_acceleration', 0.3).value
        self.max_distance = self.declare_parameter('max_goal_distance', 5.0).value
        self.position_tolerance = self.declare_parameter('position_tolerance', 0.15).value
        for value in (self.speed, self.accel, self.max_distance, self.position_tolerance):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('goal limits must be positive and finite')
        self.state = ''
        self.state_at = self.odom_at = 0.
        self.odom = None
        self.trajectory = None
        self.start_at = 0.
        self.settled_at = None
        self.goal_at = 0.
        self.hover_future = None
        self.command_pub = self.create_publisher(PositionCommand, '/px4ctrl2/command', 10)
        self.status_pub = self.create_publisher(String, '/rviz_goal/status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, '/rviz_goal/target', latched)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal, 10)
        self.odom_sub = self.create_subscription(Odometry, '/openvins/body_odom', self.on_odom, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(String, '/px4ctrl2/state', self.on_state, latched)
        self.hover_client = self.create_client(Trigger, '/px4ctrl2/hover')
        self.cancel_service = self.create_service(Trigger, '/rviz_goal/cancel', self.cancel)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('RViz goals ready: /goal_pose in global; preserves height, max speed 0.4 m/s by default')

    def notify(self, text, warning=False):
        # rclpy Humble caches severity by source location. Keep distinct call
        # sites for warning and info, including when rejecting then accepting.
        if warning:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def on_odom(self, msg):
        self.odom, self.odom_at = msg, time.monotonic()

    def on_state(self, msg):
        self.state, self.state_at = msg.data, time.monotonic()
        if self.state not in ('AUTO_HOVER', 'CMD_CTRL'):
            self.trajectory = None
            self.settled_at = None

    def feedback_valid(self):
        now = time.monotonic()
        return (self.odom is not None and now-self.odom_at < 0.5
                and now-self.state_at < 0.5 and self.state in ('AUTO_HOVER', 'CMD_CTRL')
                and self.odom.header.frame_id == 'global')

    def goal(self, msg):
        if not self.feedback_valid() or self.hover_future is not None:
            self.notify('Goal ignored: take off and wait for AUTO_HOVER with fresh odometry', True)
            return
        if msg.header.frame_id != 'global':
            self.notify('Goal ignored: RViz Fixed Frame must be global', True)
            return
        now = time.monotonic()
        p = self.odom.pose.pose.position
        try:
            current = np.array([p.x, p.y, p.z, yaw(self.odom.pose.pose.orientation)])
            # A 2D click selects x/y; the active flight height is retained.
            height = self.trajectory.sample(self.trajectory.duration)[0][2] if self.trajectory else p.z
            target = np.array([msg.pose.position.x, msg.pose.position.y, height, yaw(msg.pose.orientation)])
            if not np.isfinite(target).all() or np.linalg.norm(target[:3]-current[:3]) > self.max_distance:
                raise ValueError(f'goal must be finite and within {self.max_distance:.1f} m of the vehicle')
            if self.trajectory:
                start, velocity, acceleration = self.trajectory.sample(now-self.start_at)
            else:
                start, velocity, acceleration = current, np.zeros(4), np.zeros(4)
            trajectory = GoalTrajectory(start, velocity, acceleration, target, self.speed, self.accel)
        except ValueError as error:
            self.notify(f'Goal ignored: {error}', True)
            return
        self.trajectory, self.start_at = trajectory, now
        self.goal_at, self.settled_at = now, None
        marker = Marker()
        marker.header.frame_id = 'global'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = 'target', 0, Marker.SPHERE, Marker.ADD
        marker.pose.position = Point(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.a = 1., 0.5, 1.
        self.marker_pub.publish(marker)
        self.notify(f'Target x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f}; duration {trajectory.duration:.1f}s')

    def cancel(self, request, response):
        self.trajectory, self.settled_at = None, None
        response.success, response.message = True, 'RViz goal stream cancelled'
        return response

    def request_hover(self, reason):
        self.trajectory, self.settled_at = None, None
        self.notify(reason)
        if self.hover_client.service_is_ready():
            self.hover_future = self.hover_client.call_async(Trigger.Request())

    def tick(self):
        if self.hover_future is not None:
            if self.hover_future.done():
                try:
                    result = self.hover_future.result()
                    if not result.success:
                        self.get_logger().warning(result.message)
                except Exception as error:
                    self.get_logger().warning(str(error))
                self.hover_future = None
            return
        if self.trajectory is None:
            return
        if not self.feedback_valid():
            self.trajectory = None
            self.notify('Goal cancelled: controller state/odometry unavailable', True)
            return
        now = time.monotonic()
        elapsed = now-self.start_at
        p, v, a = self.trajectory.sample(elapsed)
        out = PositionCommand()
        out.header.frame_id = 'global'
        out.header.stamp = self.get_clock().now().to_msg()
        out.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        out.velocity.x, out.velocity.y, out.velocity.z = map(float, v[:3])
        out.acceleration.x, out.acceleration.y, out.acceleration.z = map(float, a[:3])
        out.yaw, out.yaw_dot = float(p[3]), float(v[3])
        self.command_pub.publish(out)
        actual = self.odom.pose.pose.position
        actual_v = self.odom.twist.twist.linear
        error = np.linalg.norm(np.array([actual.x, actual.y, actual.z])-p[:3])
        speed = math.sqrt(actual_v.x**2+actual_v.y**2+actual_v.z**2)
        yaw_error = abs(math.atan2(math.sin(yaw(self.odom.pose.pose.orientation)-p[3]),
                                   math.cos(yaw(self.odom.pose.pose.orientation)-p[3])))
        if elapsed >= self.trajectory.duration and error < self.position_tolerance and speed < 0.15 and yaw_error < 0.15:
            self.settled_at = self.settled_at or now
            if now-self.settled_at >= 1.:
                self.request_hover('Target reached; switching to AUTO_HOVER')
        else:
            self.settled_at = None
        if self.trajectory and elapsed > self.trajectory.duration+30:
            self.request_hover('Goal timeout; holding current position')


def main():
    rclpy.init()
    node = GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

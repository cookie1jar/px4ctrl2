#!/usr/bin/env python3
"""RViz planar/3D goals -> px4ctrl2 references in a configured VIO/LIO frame."""
from copy import deepcopy
import math
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from px4ctrl2.msg import PositionCommand
from goal_trajectory import GoalTrajectory
from rviz_goal_marker import make_goal_marker


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
        self.simulation_feedback = self.declare_parameter('simulation_feedback', False).value
        self.simulation_stamp = None
        self.simulation_at = 0.
        self.simulation_fault = False
        if self.simulation_feedback:
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.ego_mode = self.declare_parameter('use_ego_planner', False).value
        self.frame = self.declare_parameter('world_frame', 'vio_odom').value
        self.odom_topic = self.declare_parameter('odom_topic', '/openvins/body_odom').value
        self.localization_source = self.declare_parameter('localization_source', 'openvins').value
        if self.localization_source not in ('openvins', 'fastlio'):raise ValueError('Invalid localization source')
        self.lio_health = None; self.lio_health_at = 0.
        if self.localization_source == 'fastlio':
            self.create_subscription(String, '/fastlio/status', self.on_lio_health, 10)
        self.ego_goal = None
        self.ego_command = None
        self.ego_command_at = self.ego_map_at = 0.
        self.ego_map_stamp = 0.
        self.ego_map_timeout = 2.0  # depth is throttled in sensor time
        self.ego_seen_id = self.ego_reject_id = -1
        self.vio_min_features = self.declare_parameter('vio_min_features', 6 if self.ego_mode else 0).value
        if not isinstance(self.vio_min_features, int) or self.vio_min_features < 0:
            raise ValueError('vio_min_features must be a nonnegative integer')
        self.vio_features = {}
        self.vio_weak_since = None
        self.last_visual_support_at = None
        self.progress_at = 0.
        if self.ego_mode:
            from quadrotor_msgs.msg import PositionCommand as PlannerCommand
            self.ego_goal_pub = self.create_publisher(PoseStamped, '/ego/goal', 10)
            self.ego_cancel_pub = self.create_publisher(Empty, '/ego/cancel', 10)
            self.ego_command_sub = self.create_subscription(PlannerCommand, '/ego/position_cmd', self.on_ego_command, 10)
            self.ego_pose_sub = self.create_subscription(PoseStamped, '/ego/camera_pose', self.on_ego_pose, 5)
            self.ego_status_sub = self.create_subscription(String, '/ego/planner_status', self.on_ego_status, 10)
            self.vio_feature_subs = [self.create_subscription(
                PointCloud2, f'/ov_msckf/points_{kind}',
                lambda msg, kind=kind: self.on_vio_features(kind, msg), qos_profile_sensor_data)
                for kind in ('msckf','slam')] if self.localization_source == 'openvins' else []
        self.speed = self.declare_parameter('max_speed', 0.4).value
        self.accel = self.declare_parameter('max_acceleration', 0.3).value
        self.max_distance = self.declare_parameter('max_goal_distance', 5.0).value
        self.position_tolerance = self.declare_parameter('position_tolerance', 0.15).value
        self.enforce_goal_bounds = self.declare_parameter('enforce_goal_bounds', False).value
        self.goal_min = np.array(self.declare_parameter('goal_min_xyz', [-1e6,-1e6,-1e6]).value)
        self.goal_max = np.array(self.declare_parameter('goal_max_xyz', [1e6,1e6,1e6]).value)
        if (self.goal_min.shape != (3,) or self.goal_max.shape != (3,)
                or not np.isfinite(np.r_[self.goal_min,self.goal_max]).all()
                or np.any(self.goal_min >= self.goal_max)):
            raise ValueError('goal bounds require three finite ordered coordinates')
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
        self.editable_initialized = False
        self.marker_server = InteractiveMarkerServer(self, '/rviz_goal/interactive_target')
        self.marker_menu = MenuHandler()
        self.marker_menu.insert('Fly to target', callback=self.submit_marker)
        self.marker_menu.insert('Reset to vehicle', callback=self.reset_marker)
        self.command_pub = self.create_publisher(PositionCommand, '/px4ctrl2/command', 10)
        self.status_pub = self.create_publisher(String, '/rviz_goal/status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, '/rviz_goal/target', latched)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal, 10)
        self.goal_3d_sub = self.create_subscription(PoseStamped, '/goal_pose_3d', self.goal_3d, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(String, '/px4ctrl2/state', self.on_state, latched)
        self.hover_client = self.create_client(Trigger, '/px4ctrl2/hover')
        self.cancel_service = self.create_service(Trigger, '/rviz_goal/cancel', self.cancel)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('RViz goals ready: planar /goal_pose, full xyz /goal_pose_3d; drag 3D marker then choose Fly to target')

    def reset_marker(self, feedback=None):
        if not self.feedback_valid():
            self.notify('Marker reset ignored: take off and wait for fresh hover feedback', True)
            return
        pose = deepcopy(self.odom.pose.pose)
        heading = yaw(pose.orientation)
        pose.orientation.x = pose.orientation.y = 0.
        pose.orientation.z, pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        height_range = (self.goal_min[2],self.goal_max[2]) if self.enforce_goal_bounds else None
        self.marker_server.insert(make_goal_marker(pose, self.frame, height_range), feedback_callback=self.edit_marker)
        self.marker_menu.apply(self.marker_server, 'flight_target')
        self.marker_server.applyChanges()
        self.editable_initialized = True

    def edit_marker(self, feedback):
        # POSE_UPDATE and MOUSE_UP never emit a flight command.
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            p = feedback.pose.position
            self.notify(f'Draft target x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}; right-click -> Fly to target to submit')

    def submit_marker(self, feedback):
        msg = PoseStamped()
        msg.header = deepcopy(feedback.header)
        msg.pose = deepcopy(feedback.pose)
        self.goal_3d(msg)

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
            if self.ego_mode and self.ego_goal is not None:
                self.ego_cancel_pub.publish(Empty())
            self.trajectory = None
            self.ego_goal = self.ego_command = None
            self.settled_at = None
            if self.editable_initialized:
                self.marker_server.erase('flight_target')
                self.marker_server.applyChanges()
                self.editable_initialized = False

    def feedback_valid(self):
        now = time.monotonic()
        stamp = (self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9) if self.odom else 0.
        return (self.odom is not None and self.sample_fresh(self.odom_at, stamp, .5)
                and now-self.state_at < (2. if self.simulation_feedback else .5) and self.state in ('AUTO_HOVER', 'CMD_CTRL')
                and self.odom.header.frame_id == self.frame)

    def on_clock(self, msg):
        stamp = msg.clock.sec+msg.clock.nanosec*1e-9
        if self.simulation_stamp is not None and stamp < self.simulation_stamp:
            self.simulation_fault = True
        if stamp != self.simulation_stamp:
            self.simulation_at = time.monotonic()
        self.simulation_stamp = stamp

    def sample_fresh(self, received_at, stamp, timeout):
        now = time.monotonic()
        if not self.simulation_feedback:
            return 0 <= now-received_at < timeout
        return (not self.simulation_fault and self.simulation_stamp is not None
                and 0 <= now-received_at < 2. and 0 <= now-self.simulation_at < 2.
                and -.1 <= self.simulation_stamp-stamp < timeout)

    def reference_time(self):
        # Reference velocities are metres per sensor second, even when Gazebo
        # runs slower than wall time. Simulation freshness also checks /clock.
        return self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9

    def goal(self, msg):
        self.accept_goal(msg, use_target_height=False)

    def goal_3d(self, msg):
        self.accept_goal(msg, use_target_height=True)

    def accept_goal(self, msg, use_target_height):
        if not self.feedback_valid() or self.hover_future is not None:
            self.notify('Goal ignored: take off and wait for AUTO_HOVER with fresh odometry', True)
            return
        if msg.header.frame_id != self.frame:
            self.notify(f'Goal ignored: RViz Fixed Frame must be {self.frame}', True)
            return
        now = time.monotonic()
        p = self.odom.pose.pose.position
        try:
            current = np.array([p.x, p.y, p.z, yaw(self.odom.pose.pose.orientation)])
            # A 2D click selects x/y; the active flight height is retained.
            height = (msg.pose.position.z if use_target_height else
                      self.trajectory.sample(self.trajectory.duration)[0][2] if self.trajectory else p.z)
            target = np.array([msg.pose.position.x, msg.pose.position.y, height, yaw(msg.pose.orientation)])
            if not np.isfinite(target).all() or np.linalg.norm(target[:3]-current[:3]) > self.max_distance:
                raise ValueError(f'goal must be finite and within {self.max_distance:.1f} m of the vehicle')
            if self.enforce_goal_bounds:
                if not self.goal_min[2] <= target[2] <= self.goal_max[2]:
                    raise ValueError(f'目标 Z={target[2]:.2f} m 超出导航高度范围 '
                                     f'[{self.goal_min[2]:.2f}, {self.goal_max[2]:.2f}] m（{self.frame}）；目标未提交')
                if np.any(target[:2] < self.goal_min[:2]) or np.any(target[:2] > self.goal_max[:2]):
                    raise ValueError(f'目标 XY 超出规划地图范围（{self.frame}）；目标未提交')
            if self.ego_mode:
                if not self.sample_fresh(self.ego_map_at, self.ego_map_stamp, self.ego_map_timeout):
                    raise ValueError('EGO sensor/map input is stale')
                if not self.visual_support(allow_grace=True):
                    raise ValueError('FAST-LIO matching support is insufficient' if self.localization_source == 'fastlio' else 'OpenVINS visual support is insufficient; wait for features before navigating')
                distance = np.linalg.norm(target[:3]-current[:3])
                velocity = self.odom.twist.twist.linear
                if (self.state == 'AUTO_HOVER' and distance < self.position_tolerance
                        and np.linalg.norm([velocity.x, velocity.y, velocity.z]) < 0.15):
                    self.publish_target(target)
                    self.request_hover('EGO target reached: already within position tolerance; switching to AUTO_HOVER')
                    return
                goal = deepcopy(msg)
                goal.pose.position.z = float(height)
                self.ego_goal = target
                self.ego_command = None
                self.ego_reject_id = self.ego_seen_id
                self.start_at = self.reference_time()
                self.settled_at = None
                self.vio_weak_since = None
                self.progress_at = self.reference_time()
                self.ego_progress_position = current[:3].copy()
                self.ego_progress_time = self.reference_time()
                self.ego_best_distance = distance
                self.ego_approach_time = self.reference_time()
                self.ego_goal_pub.publish(goal)
                self.publish_target(target)
                self.notify(f'EGO goal x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f}; waiting for a new planned trajectory')
                return
            if self.trajectory:
                start, velocity, acceleration = self.trajectory.sample(self.reference_time()-self.start_at)
            else:
                start, velocity, acceleration = current, np.zeros(4), np.zeros(4)
            trajectory = GoalTrajectory(start, velocity, acceleration, target, self.speed, self.accel)
        except ValueError as error:
            self.notify(f'Goal ignored: {error}', True)
            return
        self.trajectory, self.start_at = trajectory, self.reference_time()
        self.goal_at, self.settled_at = now, None
        self.publish_target(target)
        self.notify(f'Target x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f}; duration {trajectory.duration:.1f}s')

    def publish_target(self, target):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = 'target', 0, Marker.SPHERE, Marker.ADD
        marker.pose.position = Point(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.a = 1., 0.5, 1.
        self.marker_pub.publish(marker)

    def cancel(self, request, response):
        if self.ego_mode:
            self.ego_cancel_pub.publish(Empty())
        self.trajectory, self.settled_at = None, None
        self.ego_goal = self.ego_command = None
        response.success, response.message = True, 'RViz goal stream cancelled'
        return response

    def request_hover(self, reason):
        if self.ego_mode:
            self.ego_cancel_pub.publish(Empty())
        self.trajectory, self.settled_at = None, None
        self.ego_goal = self.ego_command = None
        self.notify(reason)
        if self.hover_client.service_is_ready():
            self.hover_future = self.hover_client.call_async(Trigger.Request())

    def tick(self):
        if not self.editable_initialized and self.feedback_valid():
            self.reset_marker()
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
        if self.ego_mode:
            self.tick_ego()
            return
        if self.trajectory is None:
            return
        if not self.feedback_valid():
            self.trajectory = None
            self.notify('Goal cancelled: controller state/odometry unavailable', True)
            return
        now = time.monotonic()
        elapsed = self.reference_time()-self.start_at
        p, v, a = self.trajectory.sample(elapsed)
        out = PositionCommand()
        out.header.frame_id = self.frame
        out.header.stamp = self.get_clock().now().to_msg()
        if self.simulation_feedback:
            out.header.stamp = deepcopy(self.odom.header.stamp)
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

    def on_ego_pose(self, msg):
        self.ego_map_at = time.monotonic()
        self.ego_map_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def on_ego_status(self, msg):
        if self.ego_goal is not None and msg.data.startswith('REJECTED:'):
            self.request_hover('EGO goal rejected: '+msg.data.removeprefix('REJECTED:').strip())

    def on_vio_features(self, kind, msg):
        self.vio_features[kind] = (time.monotonic(), msg.width*msg.height)

    def on_lio_health(self, msg):
        self.lio_health = json.loads(msg.data); self.lio_health_at = time.monotonic()

    def visual_support(self, allow_grace=False):
        if self.localization_source == 'fastlio':
            return bool(self.lio_health and self.lio_health.get('ready')
                        and self.sample_fresh(self.lio_health_at, self.lio_health.get('stamp',0.), .5))
        if self.vio_min_features == 0:
            return True
        now = time.monotonic()
        fresh = [value[1] for value in self.vio_features.values()
                 if now-value[0] <= 1.5]
        supported = bool(fresh) and sum(fresh) >= self.vio_min_features
        if supported:
            self.last_visual_support_at = now
            return True
        # Point-cloud visualization is conditional. A single empty DDS sample
        # should not reject a target while OpenVINS still has landmarks.
        return bool(allow_grace and self.last_visual_support_at is not None
                    and now-self.last_visual_support_at <= 3.0)

    def on_ego_command(self, msg):
        self.ego_seen_id = msg.trajectory_id
        if self.ego_goal is None or msg.trajectory_id == self.ego_reject_id:
            return
        if msg.trajectory_flag != msg.TRAJECTORY_STATUS_READY or msg.header.frame_id != self.frame:
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp <= 0 or not -0.1 <= self.reference_time()-stamp <= 0.5:
            return
        values = [msg.position.x, msg.position.y, msg.position.z, msg.velocity.x,
                  msg.velocity.y, msg.velocity.z, msg.acceleration.x,
                  msg.acceleration.y, msg.acceleration.z, msg.yaw, msg.yaw_dot]
        if not all(math.isfinite(value) for value in values):
            return
        if (np.linalg.norm(values[3:6]) > 2*self.speed
                or np.linalg.norm(values[6:9]) > 2*self.accel):
            self.request_hover('EGO trajectory exceeds configured command bounds')
            return
        out = PositionCommand()
        out.header, out.position = deepcopy(msg.header), deepcopy(msg.position)
        out.velocity, out.acceleration = deepcopy(msg.velocity), deepcopy(msg.acceleration)
        out.yaw, out.yaw_dot = msg.yaw, msg.yaw_dot
        self.ego_command, self.ego_command_at = out, time.monotonic()

    def tick_ego(self):
        if self.ego_goal is None:
            return
        now = time.monotonic()
        if not self.feedback_valid() or not self.sample_fresh(self.ego_map_at, self.ego_map_stamp, self.ego_map_timeout):
            self.request_hover('EGO goal cancelled: feedback or sensor stream is stale')
            return
        if self.visual_support(allow_grace=False):
            self.vio_weak_since = None
        else:
            if self.vio_weak_since is None:
                self.vio_weak_since = self.reference_time()
            if self.reference_time()-self.vio_weak_since >= 2.:
                self.request_hover('EGO goal stopped: FAST-LIO matching support is insufficient; holding current position' if self.localization_source == 'fastlio' else 'EGO goal stopped: OpenVINS visual support is insufficient; holding current position')
                return
        if self.ego_command is None:
            if self.reference_time()-self.start_at > 10.:
                self.request_hover('EGO planning timeout; holding current position')
            return
        command_stamp = self.ego_command.header.stamp.sec+self.ego_command.header.stamp.nanosec*1e-9
        if not self.sample_fresh(self.ego_command_at, command_stamp, .5):
            self.request_hover('EGO trajectory stream stopped; holding current position')
            return
        self.command_pub.publish(self.ego_command)
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        error = np.linalg.norm(np.array([p.x,p.y,p.z])-self.ego_goal[:3])
        if error < self.ego_best_distance-0.1:
            self.ego_best_distance = error
            self.ego_approach_time = self.reference_time()
        elif (error > self.ego_best_distance+1.0
              and self.reference_time()-self.ego_approach_time >= 15.):
            self.request_hover('EGO goal stopped: trajectory is moving away from the accepted target without recovery; holding current position')
            return
        position = np.array([p.x,p.y,p.z])
        if np.linalg.norm(position-self.ego_progress_position) >= 0.15:
            self.ego_progress_position = position
            self.ego_progress_time = self.reference_time()
        elif (error > self.position_tolerance
              and self.reference_time()-self.ego_progress_time > 10.):
            self.request_hover('EGO goal stopped: no progress for 10 s; planner may be blocked; holding current position')
            return
        if self.reference_time()-self.progress_at >= 5.:
            self.notify(f'EGO progress: remaining {error:.2f} m to accepted target')
            self.progress_at = self.reference_time()
        if error < self.position_tolerance and np.linalg.norm([v.x,v.y,v.z]) < 0.15:
            self.settled_at = self.settled_at or now
            if now-self.settled_at >= 1.:
                self.request_hover('EGO target reached; switching to AUTO_HOVER')
        else:
            self.settled_at = None
        if self.ego_goal is not None and self.reference_time()-self.start_at > 120.:
            self.request_hover('EGO execution timeout; holding current position')


def main():
    rclpy.init()
    node = GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.marker_server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

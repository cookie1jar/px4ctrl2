import math
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace as Namespace
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from goal_trajectory import GoalTrajectory
from rviz_goal_node import GoalNode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class TrajectoryTest(unittest.TestCase):
    def test_limits_and_stopped_endpoints(self):
        trajectory = GoalTrajectory([0, 0, 1, 3.1], [0]*4, [0]*4, [2, 1, 1, -3.1])
        first = trajectory.sample(0)
        last = trajectory.sample(trajectory.duration)
        np.testing.assert_allclose(first[0], [0, 0, 1, 3.1], atol=1e-10)
        np.testing.assert_allclose(last[0][:3], [2, 1, 1], atol=1e-10)
        self.assertLess(abs(last[0][3]-first[0][3]), 0.1)
        for v in [first[1], first[2], last[1], last[2]]:
            np.testing.assert_allclose(v, 0, atol=1e-10)
        for t in np.linspace(0, trajectory.duration, 1001):
            p, v, a = trajectory.sample(t)
            self.assertLessEqual(np.linalg.norm(v[:3]), 0.401)
            self.assertLessEqual(np.linalg.norm(a[:3]), 0.301)
            self.assertAlmostEqual(p[2], 1.)

    def test_retarget_preserves_position_velocity_acceleration(self):
        first = GoalTrajectory([0, 0, 1, 0], [0]*4, [0]*4, [2, 0, 1, 0])
        reference = first.sample(first.duration/2)
        second = GoalTrajectory(*reference, [1, 1, 1, 0.2])
        for expected, actual in zip(reference, second.sample(0)):
            np.testing.assert_allclose(actual, expected, atol=1e-10)


class Harness(GoalNode):
    """Exercise goal handling and command generation without DDS or a vehicle."""
    def __init__(self):
        self.ego_mode = False
        self.ego_map_timeout = 2.
        self.ego_goal = self.ego_command = None
        self.speed, self.accel, self.max_distance, self.position_tolerance = 0.4, 0.3, 5., 0.15
        self.state = 'AUTO_HOVER'
        self.state_at = self.odom_at = time.monotonic()
        self.odom = Odometry()
        self.odom.header.frame_id = 'global'
        self.odom.pose.pose.position.z = 1.2
        self.odom.pose.pose.orientation.w = 1.
        self.trajectory = None
        self.editable_initialized = True
        self.marker_server = Namespace(erase=lambda name: None, applyChanges=lambda: None)
        self.start_at = self.goal_at = 0.
        self.settled_at = self.hover_future = None
        self.commands, self.markers = [], []
        self.command_pub = Namespace(publish=self.commands.append)
        self.marker_pub = Namespace(publish=self.markers.append)
        self.status_pub = Namespace(publish=lambda msg: None)
        self.hover_client = Namespace(service_is_ready=lambda: False)

    def get_logger(self):
        return Namespace(info=lambda text: None, warning=lambda text: None)

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return Namespace(now=lambda: Namespace(to_msg=lambda: Time(sec=1)))

    def reference_time(self):
        # Legacy cases use one clock; the slow-simulation regression below
        # separates this reference from wall-clock reception watchdogs.
        return time.monotonic()


class GoalHandlingTest(unittest.TestCase):
    def make_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'global'
        msg.pose.position.x = 1.
        msg.pose.orientation.w = 1.
        return msg

    def test_2d_goal_keeps_height_and_arrival_stops_stream(self):
        node = Harness()
        node.goal(self.make_goal())
        self.assertIsNotNone(node.trajectory)
        p, _, _ = node.trajectory.sample(node.trajectory.duration)
        self.assertAlmostEqual(p[2], 1.2)
        node.tick()
        self.assertEqual(node.commands[-1].header.frame_id, 'global')
        self.assertAlmostEqual(node.commands[-1].position.z, 1.2)
        node.odom.pose.pose.position.x = 1.
        node.state = 'CMD_CTRL'
        end = node.start_at+node.trajectory.duration
        for now in [end+0.1, end+1.2]:
            node.state_at = node.odom_at = now
            with patch('rviz_goal_node.time.monotonic', return_value=now):
                node.tick()
        self.assertIsNone(node.trajectory)

    def test_invalid_goals_and_non_hover_states_rejected(self):
        for setting in ['frame', 'distance', 'nan', 'not_flying', 'stale']:
            with self.subTest(setting=setting):
                node, goal = Harness(), self.make_goal()
                if setting == 'frame': goal.header.frame_id = 'map'
                if setting == 'distance': goal.pose.position.x = 6.
                if setting == 'nan': goal.pose.position.x = math.nan
                if setting == 'not_flying': node.state = 'MANUAL_CTRL'
                if setting == 'stale': node.odom_at -= 1.
                node.goal(goal)
                self.assertIsNone(node.trajectory)

    def test_cancel_stops_future_commands(self):
        node = Harness()
        node.goal(self.make_goal())
        from std_srvs.srv import Trigger
        response = node.cancel(None, Trigger.Response())
        node.tick()
        self.assertTrue(response.success)
        self.assertEqual(node.commands, [])

    def test_rejected_then_valid_goal_with_real_ros_logger(self):
        # Humble caches severity per source call site. A mock logger cannot
        # reproduce the crash when warning/info share the same source line.
        from rclpy.logging import get_logger
        node = Harness()
        logger = get_logger('rviz_goal_regression')
        with patch.object(node, 'get_logger', return_value=logger):
            invalid = self.make_goal()
            invalid.pose.position.x = 6.
            node.goal(invalid)
            self.assertIsNone(node.trajectory)
            node.goal(self.make_goal())
            self.assertIsNotNone(node.trajectory)
            node.goal(invalid)
            self.assertIsNotNone(node.trajectory)

    def test_3d_goal_tracks_requested_height(self):
        node = Harness()
        goal = self.make_goal()
        goal.pose.position.z = 2.4
        node.goal_3d(goal)
        self.assertIsNotNone(node.trajectory)
        final, velocity, acceleration = node.trajectory.sample(node.trajectory.duration)
        np.testing.assert_allclose(final[:3], [1., 0., 2.4], atol=1e-9)
        mid = node.start_at+node.trajectory.duration/2
        node.odom_at = node.state_at = mid
        with patch('rviz_goal_node.time.monotonic', return_value=mid):
            node.tick()
        self.assertGreater(node.commands[-1].position.z, 1.2)
        self.assertGreater(node.commands[-1].velocity.z, 0.)

    def test_3d_nan_height_rejected(self):
        node = Harness()
        goal = self.make_goal()
        goal.pose.position.z = math.nan
        node.goal_3d(goal)
        self.assertIsNone(node.trajectory)

    def test_slow_simulation_progresses_in_sensor_seconds(self):
        node = Harness()
        node.odom.header.stamp.sec = 100
        node.reference_time = lambda: GoalNode.reference_time(node)
        goal = self.make_goal()
        goal.pose.position.x = 2.
        node.goal_3d(goal)
        self.assertEqual(node.start_at, 100.)
        wall = time.monotonic()+5.
        node.state_at = node.odom_at = wall
        node.odom.header.stamp.sec = 101  # 0.2x simulation speed
        with patch('rviz_goal_node.time.monotonic', return_value=wall):
            node.tick()
        expected = node.trajectory.sample(1.)
        self.assertAlmostEqual(node.commands[-1].position.x, expected[0][0])
        self.assertAlmostEqual(node.commands[-1].velocity.x, expected[1][0])
        self.assertLess(node.commands[-1].position.x, node.trajectory.sample(5.)[0][0])
        # Retarget at the same sensor time, despite a later wall clock.
        wall += 0.2
        node.state_at = node.odom_at = wall
        with patch('rviz_goal_node.time.monotonic', return_value=wall):
            node.goal(self.make_goal())
        np.testing.assert_allclose(node.trajectory.sample(0.)[0], expected[0], atol=1e-9)
        # Reception still times out on real time if simulation stops.
        with patch('rviz_goal_node.time.monotonic', return_value=wall+0.6):
            node.tick()
        self.assertIsNone(node.trajectory)

    def test_ego_mode_routes_goal_and_blocks_old_or_stale_commands(self):
        node=Harness();node.ego_mode=True
        node.reference_time=lambda:100.
        node.ego_goal=node.ego_command=None
        node.ego_seen_id=7;node.ego_map_at=time.monotonic()
        goals=[];node.ego_goal_pub=Namespace(publish=goals.append)
        node.goal_3d(self.make_goal())
        self.assertEqual(len(goals),1)
        self.assertIsNone(node.trajectory)  # No competing straight-line generator.
        from geometry_msgs.msg import Point, Vector3
        from std_msgs.msg import Header
        cmd=Namespace(trajectory_id=7,trajectory_flag=1,TRAJECTORY_STATUS_READY=1,
                      header=Header(frame_id='global'),position=Point(x=.1,z=1.2),
                      velocity=Vector3(x=.1),acceleration=Vector3(),yaw=0.,yaw_dot=0.)
        cmd.header.stamp.sec=100
        node.on_ego_command(cmd)
        self.assertIsNone(node.ego_command)
        cmd.trajectory_id=8
        node.on_ego_command(cmd);node.tick()
        self.assertEqual(len(node.commands),1)
        self.assertAlmostEqual(node.commands[0].position.x,.1)
        node.ego_command_at-=1.
        node.tick()
        self.assertIsNone(node.ego_goal)
        self.assertEqual(len(node.commands),1)

    def test_drag_only_edits_submit_starts_3d_trajectory(self):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        from rviz_goal_marker import make_goal_marker
        node = Harness()
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = 'global'
        feedback.pose.position.x = 1.
        feedback.pose.position.z = 2.
        feedback.pose.orientation.w = 1.
        for event in [feedback.POSE_UPDATE, feedback.MOUSE_UP]:
            feedback.event_type = event
            node.edit_marker(feedback)
            self.assertIsNone(node.trajectory)
            self.assertEqual(node.commands, [])
        node.submit_marker(feedback)
        self.assertAlmostEqual(node.trajectory.sample(node.trajectory.duration)[0][2], 2.)
        marker = make_goal_marker(feedback.pose)
        self.assertEqual(marker.header.frame_id, 'global')
        self.assertEqual([c.name for c in marker.controls],
                         ['target_menu', 'move_x', 'move_y', 'move_z', 'rotate_yaw'])
        node.on_state(Namespace(data='AUTO_LAND'))
        self.assertIsNone(node.trajectory)
        self.assertFalse(node.editable_initialized)


if __name__ == '__main__':
    unittest.main()

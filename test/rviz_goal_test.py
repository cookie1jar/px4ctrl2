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
        self.speed, self.accel, self.max_distance, self.position_tolerance = 0.4, 0.3, 5., 0.15
        self.state = 'AUTO_HOVER'
        self.state_at = self.odom_at = time.monotonic()
        self.odom = Odometry()
        self.odom.header.frame_id = 'global'
        self.odom.pose.pose.position.z = 1.2
        self.odom.pose.pose.orientation.w = 1.
        self.trajectory = None
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


if __name__ == '__main__':
    unittest.main()

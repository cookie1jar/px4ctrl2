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
        self.simulation_feedback=False
        self.simulation_stamp=None;self.simulation_at=0.;self.simulation_fault=False
        self.ego_map_stamp=0.
        self.enforce_goal_bounds=False
        self.goal_min=np.array([-1e6,-1e6,-1e6]);self.goal_max=-self.goal_min
        self.frame = 'vio_odom'
        self.localization_source = 'openvins'
        self.ego_mode = False
        self.ego_map_timeout = 2.
        self.ego_goal = self.ego_command = None
        self.vio_min_features = 0
        self.vio_features = {}
        self.vio_weak_since = None
        self.last_visual_support_at = None
        self.progress_at = 0.
        self.planner_cancels = []
        self.ego_cancel_pub = Namespace(publish=self.planner_cancels.append)
        self.speed, self.accel, self.max_distance, self.position_tolerance = 0.4, 0.3, 5., 0.15
        self.state = 'AUTO_HOVER'
        self.state_at = self.odom_at = time.monotonic()
        self.odom = Odometry()
        self.odom.header.frame_id = 'vio_odom'
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
    def test_high_lio_goal_is_accepted_inside_configured_3d_volume(self):
        node=Harness();node.ego_mode=True;node.enforce_goal_bounds=True
        node.max_distance=12.;node.goal_min=np.array([-11.99,-11.99,.5]);node.goal_max=np.array([11.99,11.99,5.])
        node.ego_seen_id=-1;node.ego_map_at=time.monotonic()
        goals=[];node.ego_goal_pub=Namespace(publish=goals.append)
        goal=self.make_goal();goal.pose.position.z=4.2
        node.goal_3d(goal)
        self.assertEqual(len(goals),1)
        self.assertEqual(goals[0].pose.position.z,4.2)

    def test_out_of_bounds_goal_cannot_replace_or_cancel_active_return(self):
        node=Harness();node.ego_mode=True;node.enforce_goal_bounds=True
        node.max_distance=12.;node.goal_min=np.array([-11.99,-11.99,.5]);node.goal_max=np.array([11.99,11.99,5.])
        node.ego_seen_id=-1;node.ego_map_at=time.monotonic()
        goals=[];node.ego_goal_pub=Namespace(publish=goals.append)
        initial=self.make_goal();initial.pose.position.z=1.2
        node.goal_3d(initial)
        original=node.ego_goal.copy()
        from px4ctrl2.msg import PositionCommand
        command=PositionCommand();node.ego_command=command;node.state='CMD_CTRL'
        for x,z in [(1.,6.),(1.,.2),(12.,1.2)]:
            goal=self.make_goal();goal.pose.position.x=x;goal.pose.position.z=z
            node.goal_3d(goal)
            np.testing.assert_array_equal(node.ego_goal,original)
            self.assertIs(node.ego_command,command)
            self.assertEqual(len(goals),1)
            self.assertFalse(node.planner_cancels)

    def test_rviz_marker_displays_height_bounds(self):
        from rviz_goal_marker import make_goal_marker
        node=Harness()
        marker=make_goal_marker(node.odom.pose.pose,'lio_odom',(.5,5.))
        self.assertIn('0.50 .. 5.00',marker.description)
        self.assertIn('lio_odom',marker.description)

    def test_simulation_feedback_uses_clock_and_retains_wall_watchdog(self):
        node=Harness();node.simulation_feedback=True
        now=time.monotonic()
        node.simulation_stamp=10.2;node.simulation_at=now
        node.odom.header.stamp.sec=10
        node.odom_at=now-.7
        self.assertTrue(node.feedback_valid())
        node.simulation_stamp=10.6
        self.assertFalse(node.feedback_valid())  # true sensor failure in physics time
        node.simulation_stamp=10.2;node.simulation_at=now-2.1
        self.assertFalse(node.feedback_valid())
        node.simulation_at=now;node.odom_at=now-2.1
        self.assertFalse(node.feedback_valid())
        node.odom_at=now-.7;node.simulation_feedback=False
        self.assertFalse(node.feedback_valid())

    def test_lio_navigation_survives_slow_clock_but_stops_for_real_sensor_loss(self):
        node=Harness();node.simulation_feedback=True;node.ego_mode=True
        node.localization_source='fastlio';node.ego_seen_id=-1
        now=time.monotonic()
        node.simulation_stamp=100.2;node.simulation_at=now
        node.odom.header.stamp.sec=100;node.odom.header.stamp.nanosec=100000000
        node.reference_time=lambda:100.1
        node.odom_at=node.ego_map_at=node.lio_health_at=now-.8
        node.ego_map_stamp=100.1
        node.lio_health={'ready':True,'stamp':100.1}
        goals=[];node.ego_goal_pub=Namespace(publish=goals.append)
        node.goal(self.make_goal());self.assertEqual(len(goals),1)
        from px4ctrl2.msg import PositionCommand
        node.ego_command=PositionCommand()
        node.ego_command.header.stamp=node.odom.header.stamp
        node.ego_command_at=now-.8
        node.tick_ego()
        self.assertIsNotNone(node.ego_goal);self.assertEqual(len(node.commands),1)
        node.simulation_stamp=100.7
        node.tick_ego()
        self.assertIsNone(node.ego_goal);self.assertEqual(len(node.commands),1)

    def make_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'vio_odom'
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
        self.assertEqual(node.commands[-1].header.frame_id, 'vio_odom')
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
        self.assertAlmostEqual(node.markers[-1].pose.position.x,1.)
        self.assertIsNone(node.trajectory)  # No competing straight-line generator.
        from geometry_msgs.msg import Point, Vector3
        from std_msgs.msg import Header
        cmd=Namespace(trajectory_id=7,trajectory_flag=1,TRAJECTORY_STATUS_READY=1,
                      header=Header(frame_id='vio_odom'),position=Point(x=.1,z=1.2),
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

    def test_lio_ego_accepts_lio_frame_without_vio_features_and_rejects_unhealthy_lio(self):
        node=Harness();node.ego_mode=True;node.frame='lio_odom'
        node.localization_source='fastlio';node.odom.header.frame_id='lio_odom'
        node.vio_min_features=6 # LIO health, not this VIO feature counter, owns the gate
        node.ego_seen_id=-1;node.ego_map_at=time.monotonic()
        node.lio_health={'ready':True};node.lio_health_at=time.monotonic()
        node.reference_time=lambda:100.
        goals=[];node.ego_goal_pub=Namespace(publish=goals.append)
        node.goal_3d(self.make_goal()) # old VIO frame must not enter LIO planner
        self.assertFalse(goals)
        goal=self.make_goal();goal.header.frame_id='lio_odom'
        node.goal_3d(goal)
        self.assertEqual(len(goals),1)
        self.assertEqual(goals[0].header.frame_id,'lio_odom')
        self.assertEqual(node.markers[-1].header.frame_id,'lio_odom')
        node.lio_health={'ready':False}
        node.goal_3d(goal);self.assertEqual(len(goals),1)
        node.lio_health={'ready':True};node.lio_health_at-=1.
        node.goal_3d(goal);self.assertEqual(len(goals),1)

    def test_ego_nearby_goal_in_hover_does_not_wait_for_a_bspline(self):
        node=Harness();node.ego_mode=True
        node.ego_map_at=time.monotonic()
        goals=[];node.ego_goal_pub=Namespace(publish=goals.append)
        goal=self.make_goal();goal.pose.position.x=.05
        goal.pose.position.z=1.2
        node.goal_3d(goal)
        self.assertFalse(goals)
        self.assertIsNone(node.ego_goal)
        self.assertEqual(len(node.planner_cancels),1)

    def test_ego_fresh_stationary_commands_cannot_hide_a_stalled_goal(self):
        node=Harness();node.ego_mode=True
        node.max_distance=12.;node.ego_seen_id=-1
        node.ego_map_at=time.monotonic()
        node.ego_goal_pub=Namespace(publish=lambda msg: None)
        reference=[100.];node.reference_time=lambda:reference[0]
        goal=self.make_goal();goal.pose.position.x=8.
        node.goal_3d(goal)
        self.assertIsNotNone(node.ego_goal)  # Long targets remain supported.
        from px4ctrl2.msg import PositionCommand
        node.ego_command=PositionCommand()
        for t,x in [(105.,.2),(114.,.25),(116.,.25)]:
            reference[0]=t
            now=time.monotonic()
            node.state_at=node.odom_at=node.ego_map_at=node.ego_command_at=now
            node.odom.pose.pose.position.x=x
            node.tick_ego()
            self.assertEqual(node.ego_goal is None,t==116.)
        self.assertEqual(len(node.planner_cancels),1)

    def test_ego_detour_keeps_running_when_distance_to_goal_increases(self):
        node=Harness();node.ego_mode=True
        node.ego_seen_id=-1;node.ego_map_at=time.monotonic()
        node.ego_goal_pub=Namespace(publish=lambda msg: None)
        reference=[100.];node.reference_time=lambda:reference[0]
        node.goal(self.make_goal())
        from px4ctrl2.msg import PositionCommand
        node.ego_command=PositionCommand()
        for t,y in [(105.,.3),(110.,.6),(115.,.9)]:
            reference[0]=t
            now=time.monotonic()
            node.state_at=node.odom_at=node.ego_map_at=node.ego_command_at=now
            node.odom.pose.pose.position.y=y
            node.tick_ego()
            self.assertIsNotNone(node.ego_goal)
        self.assertFalse(node.planner_cancels)

    def test_ego_cancels_sustained_divergence_despite_continued_motion(self):
        node=Harness();node.ego_mode=True
        node.ego_seen_id=-1;node.ego_map_at=time.monotonic()
        node.ego_goal_pub=Namespace(publish=lambda msg: None)
        reference=[100.];node.reference_time=lambda:reference[0]
        goal=self.make_goal();goal.pose.position.x=3.26
        node.goal(goal)
        from px4ctrl2.msg import PositionCommand
        node.ego_command=PositionCommand()
        notices=[];node.notify=lambda message,*args: notices.append(message)
        # Shape of the 20261020_150523 run: first approaches, then the
        # planner reference and vehicle both continue beyond the target.
        samples=[(105.,.64,.70),(110.,1.50,1.17),(115.,2.35,1.24),
                 (120.,2.90,1.06),(125.,3.72,.95),(130.,4.59,.94),
                 (135.,5.62,1.09)]
        for t,x,y in samples:
            reference[0]=t
            now=time.monotonic()
            node.state_at=node.odom_at=node.ego_map_at=node.ego_command_at=now
            node.odom.pose.pose.position.x=x;node.odom.pose.pose.position.y=y
            node.tick_ego()
            self.assertEqual(node.ego_goal is None,t==135.)
        self.assertIn('moving away',notices[-1])
        self.assertEqual(len(node.planner_cancels),1)

    def test_ego_stops_on_sustained_weak_visual_support_even_with_fresh_odom(self):
        node=Harness();node.ego_mode=True;node.vio_min_features=6
        node.ego_seen_id=-1;node.ego_map_at=time.monotonic()
        node.ego_goal_pub=Namespace(publish=lambda msg: None)
        reference=[100.];node.reference_time=lambda:reference[0]
        node.on_vio_features('msckf',Namespace(width=0,height=1))
        node.on_vio_features('slam',Namespace(width=2,height=1))
        node.goal_3d(self.make_goal())
        self.assertIsNone(node.ego_goal)
        node.on_vio_features('slam',Namespace(width=12,height=1))
        node.goal_3d(self.make_goal())
        self.assertIsNotNone(node.ego_goal)
        node.on_vio_features('slam',Namespace(width=2,height=1))
        node.tick()
        reference[0]=101.
        node.on_vio_features('slam',Namespace(width=12,height=1))
        node.tick()  # Recovery cancels the weak interval.
        self.assertIsNone(node.vio_weak_since)
        node.on_vio_features('slam',Namespace(width=2,height=1))
        node.tick()
        reference[0]=103.1
        node.tick()
        self.assertIsNone(node.ego_goal)
        self.assertEqual(node.commands,[])
        self.assertEqual(len(node.planner_cancels),1)

    def test_blocked_goal_cancels_planner_and_stops_command_gate(self):
        node=Harness();node.ego_mode=True
        node.ego_goal=np.array([1.,0.,1.2,0.])
        node.on_ego_status(Namespace(data='REJECTED: target is inside inflated obstacle or height boundary'))
        node.tick()
        self.assertIsNone(node.ego_goal)
        self.assertEqual(node.commands,[])
        self.assertEqual(len(node.planner_cancels),1)

    def test_drag_only_edits_submit_starts_3d_trajectory(self):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        from rviz_goal_marker import make_goal_marker
        node = Harness()
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = 'vio_odom'
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
        self.assertEqual(marker.header.frame_id, 'vio_odom')
        self.assertEqual([c.name for c in marker.controls],
                         ['target_menu', 'move_x', 'move_y', 'move_z', 'rotate_yaw'])
        node.on_state(Namespace(data='AUTO_LAND'))
        self.assertIsNone(node.trajectory)
        self.assertFalse(node.editable_initialized)


if __name__ == '__main__':
    unittest.main()

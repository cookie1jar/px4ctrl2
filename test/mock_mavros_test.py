#!/usr/bin/env python3
"""Protocol/state-machine test with mocked MAVROS. Does not validate flight dynamics."""
import os
import subprocess
import socket
import time
import unittest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, ExtendedState, AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4ctrl2.msg import PositionCommand


class Mock(Node):
    def __init__(self):
        super().__init__('mock_px4ctrl2_mavros')
        self.mode = 'POSCTL'
        self.armed = False
        self.z = 0.0
        self.vz = 0.0
        self.on_ground = True
        self.publish_feedback = True
        self.reject_mode = False
        self.controller_state = ''
        self.outputs = []
        self.events = []
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.ext_pub = self.create_publisher(ExtendedState, '/mavros/extended_state', 10)
        self.odom_pub = self.create_publisher(Odometry, '/test/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/mavros/imu/data', 10)
        self.cmd_pub = self.create_publisher(PositionCommand, '/px4ctrl2/command', 10)
        self.mode_srv = self.create_service(SetMode, '/mavros/set_mode', self.set_mode)
        self.arm_srv = self.create_service(CommandBool, '/mavros/cmd/arming', self.arm)
        self.output_sub = self.create_subscription(AttitudeTarget, '/mavros/setpoint_raw/attitude', self.output, 10)
        self.state_sub = self.create_subscription(String, '/px4ctrl2/state', self.state, 10)
        self.command_clients = {name: self.create_client(Trigger, '/px4ctrl2/' + name) for name in ('takeoff', 'hover', 'land')}
        self.timer = self.create_timer(0.02, self.publish)

    def output(self, msg):
        self.outputs.append(msg)

    def state(self, msg):
        self.controller_state = msg.data

    def set_mode(self, req, res):
        self.events.append(('mode', req.custom_mode))
        res.mode_sent = not self.reject_mode
        if res.mode_sent:
            self.mode = req.custom_mode
        return res

    def arm(self, req, res):
        self.events.append(('arm', req.value))
        self.armed = req.value
        res.success = True
        return res

    def publish(self):
        state = State()
        state.connected = True
        state.mode = self.mode
        state.armed = self.armed
        self.state_pub.publish(state)
        ext = ExtendedState()
        ext.landed_state = ExtendedState.LANDED_STATE_ON_GROUND if self.on_ground else ExtendedState.LANDED_STATE_IN_AIR
        self.ext_pub.publish(ext)
        if not self.publish_feedback:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.z = self.z
        odom.twist.twist.linear.z = self.vz
        self.odom_pub.publish(odom)
        imu = Imu()
        imu.header = odom.header
        imu.orientation.w = 1.0
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

    def wait(self, predicate, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def call(self, name):
        client = self.command_clients[name]
        if not client.wait_for_service(timeout_sec=5):
            raise AssertionError('service unavailable: ' + name)
        future = client.call_async(Trigger.Request())
        if not self.wait(future.done):
            raise AssertionError('service response timeout')
        return future.result()


class ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Fail before creating DDS participants in environments that prohibit UDP.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', 0))
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.mock = Mock()
        executable = os.environ['PX4CTRL2_TEST_EXECUTABLE']
        self.process = subprocess.Popen([executable, '--ros-args', '-p', 'odom_topic:=/test/odom',
                                        '-p', 'takeoff_height:=0.2', '-p', 'takeoff_land_speed:=1.0',
                                        '-p', 'feedback_timeout:=0.3'], stdout=subprocess.DEVNULL)
        self.assertTrue(self.mock.wait(lambda: self.mock.controller_state == 'MANUAL_CTRL'))
        self.mock.wait(lambda: False, 0.3)

    def tearDown(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.mock.destroy_node()

    def test_takeoff_command_timeout_land(self):
        m = self.mock
        self.assertFalse(m.armed)
        self.assertTrue(m.call('takeoff').success)
        self.assertTrue(m.wait(lambda: m.controller_state == 'AUTO_TAKEOFF'))
        self.assertLess(m.events.index(('mode', 'OFFBOARD')), m.events.index(('arm', True)))
        self.assertGreater(len(m.outputs), 50)  # Offboard prestream continues during service requests.
        m.on_ground = False
        m.wait(lambda: False, 3.1)
        m.z = 0.2
        self.assertTrue(m.wait(lambda: m.controller_state == 'AUTO_HOVER'))
        cmd = PositionCommand()
        cmd.header.frame_id = 'world'
        cmd.position.z = 0.2
        m.cmd_pub.publish(cmd)
        self.assertTrue(m.wait(lambda: m.controller_state == 'CMD_CTRL'))
        self.assertTrue(m.wait(lambda: m.controller_state == 'AUTO_HOVER'))
        self.assertTrue(m.call('land').success)
        m.z = 0.0
        m.on_ground = True
        self.assertTrue(m.wait(lambda: ('arm', False) in m.events, 6))
        self.assertTrue(m.wait(lambda: m.controller_state == 'MANUAL_CTRL'))
        self.assertFalse(m.armed)
        self.assertEqual(m.mode, 'POSCTL')

    def test_mode_rejection_aborts_without_arming(self):
        m = self.mock
        m.reject_mode = True
        self.assertTrue(m.call('takeoff').success)
        self.assertTrue(m.wait(lambda: m.controller_state == 'ABORT'))
        self.assertNotIn(('arm', True), m.events)

    def test_feedback_loss_stops_outputs(self):
        m = self.mock
        self.assertTrue(m.call('takeoff').success)
        self.assertTrue(m.wait(lambda: m.controller_state == 'AUTO_TAKEOFF'))
        m.publish_feedback = False
        self.assertTrue(m.wait(lambda: m.controller_state == 'ABORT'))
        m.wait(lambda: False, 0.2)
        count = len(m.outputs)
        m.wait(lambda: False, 0.3)
        self.assertEqual(count, len(m.outputs))
        self.assertEqual(m.events.count(('mode', 'OFFBOARD')), 1)


if __name__ == '__main__':
    unittest.main()

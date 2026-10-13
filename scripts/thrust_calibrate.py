#!/usr/bin/env python3
"""ROS 2 port of px4ctrl thrust calibration recorder; records CSV, does not fit a model."""
import csv
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped


class Recorder(Node):
    def __init__(self):
        super().__init__('px4ctrl2_thrust_calibrate')
        self.interval = float(self.declare_parameter('time_interval', 1.0).value)
        self.mass = float(self.declare_parameter('mass_kg', 1.0).value)
        self.minimum_voltage = float(self.declare_parameter('min_battery_voltage', 13.2).value)
        path = Path(self.declare_parameter('output_file', '/tmp/px4ctrl2_thrust.csv').value)
        if self.interval <= 0 or self.mass <= 0:
            raise ValueError('time_interval and mass_kg must be positive')
        self.file = path.open('a', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['unix_time', 'mass_kg', 'mean_thrust', 'mean_voltage'])
        self.recording = False
        self.voltages, self.thrusts = [], []
        self.battery_sub = self.create_subscription(BatteryState, '/mavros/battery', self.battery, qos_profile_sensor_data)
        self.thrust_sub = self.create_subscription(AttitudeTarget, '/mavros/setpoint_raw/attitude', self.thrust, 10)
        self.trigger_sub = self.create_subscription(PoseStamped, '/traj_start_trigger', self.trigger, 10)
        self.timer = self.create_timer(self.interval, self.flush)
        self.get_logger().info(f'Writing {path}; trajectory trigger toggles recording')

    def trigger(self, _):
        self.flush()
        self.recording = not self.recording
        self.get_logger().info('Recording' if self.recording else 'Paused')

    def battery(self, msg):
        if self.recording and msg.voltage >= self.minimum_voltage:
            self.voltages.append(float(msg.voltage))

    def thrust(self, msg):
        if self.recording and 0.0 <= msg.thrust <= 1.0:
            self.thrusts.append(float(msg.thrust))

    def flush(self):
        if self.voltages and self.thrusts:
            self.writer.writerow([time.time(), self.mass,
                                  sum(self.thrusts)/len(self.thrusts),
                                  sum(self.voltages)/len(self.voltages)])
            self.file.flush()
        self.voltages.clear()
        self.thrusts.clear()


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    finally:
        node.flush()
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

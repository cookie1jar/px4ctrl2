from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('px4ctrl2'), 'config', 'ctrl.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('odom_topic', default_value='/mavros/local_position/odom'),
        DeclareLaunchArgument('odom_reference_time', default_value='false'),
        Node(package='px4ctrl2', executable='px4ctrl2_node', name='px4ctrl2',
             output='screen', parameters=[LaunchConfiguration('config'),
             {'odom_topic': LaunchConfiguration('odom_topic'),
              'odom_reference_time': ParameterValue(LaunchConfiguration('odom_reference_time'), value_type=bool)}])])

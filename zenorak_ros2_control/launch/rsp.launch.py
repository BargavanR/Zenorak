import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('zenorak_ros2_control'))
    xacro_file = os.path.join(pkg_path,'description','zenorak.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_description_config}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    return LaunchDescription([
            node_robot_state_publisher
        ])
    
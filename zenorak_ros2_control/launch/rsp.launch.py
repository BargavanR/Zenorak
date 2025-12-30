import os   # Provides functions to handle file and directory paths

from ament_index_python.packages import get_package_share_directory  
# Utility to get the absolute path of a package’s shared directory (used for locating files like URDF/Xacro, configs, etc.)

from launch import LaunchDescription  
# Defines the structure of a ROS 2 launch file

from launch.substitutions import LaunchConfiguration, Command  
# LaunchConfiguration allows runtime arguments; Command can execute shell commands within launch files

from launch.actions import DeclareLaunchArgument  
# Lets us declare user-configurable arguments when launching (not directly used here but imported for flexibility)

# Ensure ParameterValue is available so we can mark parameters as raw strings
from launch_ros.actions import Node  
from launch_ros.parameter_descriptions import ParameterValue
# Describes and launches a ROS 2 node

import xacro  
# Library to process Xacro files (XML Macros), which generate URDF descriptions dynamically


def generate_launch_description():
    # Define the absolute path to the package that contains robot description
    pkg_path = os.path.join(get_package_share_directory('zenorak_ros2_control'))

    # Locate the robot description Xacro file inside the package
    xacro_file = os.path.join(pkg_path,'description','zenorak.xacro')

    # Process the Xacro file into a full URDF XML string
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Store the URDF in a dictionary format for ROS parameters
    # Wrap the URDF string in ParameterValue with value_type=str so the launch
    # system does not attempt to parse the XML as YAML (which causes the
    # "Unable to parse the value of parameter robot_description as yaml" error).
    params = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # Launch the robot_state_publisher node
    # - Publishes the kinematic transforms (TF) and robot description to the rest of ROS
    # - Essential for visualization in RViz and for other nodes that depend on robot structure
    node_robot_state_publisher = Node(
        package='robot_state_publisher',     # ROS 2 package
        executable='robot_state_publisher',  # Node executable name
        output='screen',                     # Print node logs to console
        parameters=[params]                  # Pass robot description as parameter
    )

    # Build and return the launch description containing our single node
    return LaunchDescription([
            node_robot_state_publisher
        ])

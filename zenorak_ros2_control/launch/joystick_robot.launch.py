import os
# Import the standard Python library 'os' for file path manipulations.

from ament_index_python.packages import get_package_share_directory
# Import ROS 2 utility to get the path of a package's share directory, useful for accessing launch files, URDFs, and config files.

from launch import LaunchDescription
# Import LaunchDescription to define the overall launch description for ROS 2.

from launch.actions import IncludeLaunchDescription, TimerAction
# IncludeLaunchDescription: allows including another launch file within this launch.
# TimerAction: delays the execution of specified actions by a certain period of time.

from launch.launch_description_sources import PythonLaunchDescriptionSource
# Provides a source for a Python-based launch file to be included via IncludeLaunchDescription.

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
# Command substitution allows running a shell command and using its output as a launch parameter (like fetching robot_description).

from launch.actions import RegisterEventHandler
# RegisterEventHandler allows executing actions based on certain events in the launch lifecycle (e.g., process start).

from launch.event_handlers import OnProcessStart
# OnProcessStart triggers an action when a specific process starts, useful for spawning controllers after the ros2_control_node starts.

from launch_ros.actions import Node
# Node action launches a ROS 2 node from a specific package and executable.

def generate_launch_description():
    # Main function required by ROS 2 launch files to return the launch description.
    
    package_name='zenorak_ros2_control' 
    # Store the ROS 2 package name in a variable for easier reuse in file paths.

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), 
    )
    # Include the robot_state_publisher launch file (rsp.launch.py) from the zenorak_ros2_control package.
    # This will start the robot_state_publisher node that publishes the robot's TFs and URDF.

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )
    # Commented out: optional joystick launch inclusion.  
    # When uncommented, it launches joystick input nodes to drive the robot manually.

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )
    # Commented out: optional twist_mux node for combining multiple cmd_vel sources.  
    # Useful when multiple inputs (joystick, autonomous planner) need to be merged to control the robot.

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    # Fetches the 'robot_description' parameter from the running robot_state_publisher node.  
    # This ensures the ros2_control_node has the same URDF currently published.

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','motor_controller.yaml')
    # Path to the controller YAML file which contains configuration for all controllers (diff drive, joint broadcaster, etc.).
    actuator_controller_params = os.path.join(
    get_package_share_directory(package_name),
    'config',
    'actuator_controller.yaml'
    )
    joy_yaml = os.path.join(
        get_package_share_directory('zenorak_ros2_control'),
        'config',
        'joystick.yaml'
    )

    # 1️⃣ Joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_yaml]
    )

    # 2️⃣ Base / motor teleop
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',   # IMPORTANT
        parameters=[joy_yaml],
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )


    # # 3️⃣ Arm / manipulator teleop
    # arm_node = Node(
    #     package='zenorak_ros2_control',
    #     executable='joy_control.py',
    #     output='screen'  # Optional: log joint updates to console
    # )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controller_params_file,          # motor controllers
            actuator_controller_params       # actuator controllers
        ],
        output="screen"
    )

    # Launch the ros2_control_node with the robot_description and controller parameters.  
    # This node manages all hardware interfaces and controllers for the robot.

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    # Delay the startup of the ros2_control_node by 3 seconds to ensure that robot_state_publisher is running.  
    # This avoids race conditions where the robot_description is not yet available.

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    # Node to spawn the differential drive controller using the controller_manager spawner tool.

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )
    # Ensure the differential drive controller is spawned **only after** the ros2_control_node has started.  
    # This avoids errors from trying to spawn a controller before the hardware is initialized.

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    # Node to spawn the joint state broadcaster controller, which publishes the positions of all robot joints.

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    arm_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "arm_trajectory_controller",
        "--controller-manager",
        "/controller_manager"
    ],
    )
    delayed_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[arm_controller_spawner],
        )
    )



    
    # Ensure the joint state broadcaster is spawned after the ros2_control_node starts, similar to diff_drive_spawner.

    # Launch them all!
    return LaunchDescription([
        rsp,  # Include robot_state_publisher launch
        # joystick,  # Optional joystick input (commented out)
        # twist_mux,  # Optional twist mux node (commented out)
        joy_node,
        teleop_node,
        # arm_node,
        delayed_controller_manager,  # Launch the ros2_control_node with delay
        delayed_diff_drive_spawner,  # Spawn differential drive controller after ros2_control_node starts
        delayed_joint_broad_spawner , # Spawn joint state broadcaster after ros2_control_node starts ,
        delayed_arm_controller_spawner
    ])

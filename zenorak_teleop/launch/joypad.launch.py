from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Start the joystick driver (joy_node) and our translator
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    joy_teleop = Node(
        package='zenorak_teleop',
        executable='joy_teleop.py',
        name='joy_teleop',
        output='screen',
        parameters=[{
            'linear_scale': 1.0,
            'angular_scale': 0.5,
            'deadzone': 0.05,
            'enable_button': 6,
            'manip_enable_button': 7,
            'manip_buttons': [3,1,2,0],
            'manip_buttons_alt': []
        }]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop
    ])

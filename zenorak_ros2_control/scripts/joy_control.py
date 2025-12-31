#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JoyButtonArmTeleopHold(Node):
    def __init__(self):
        super().__init__('joy_button_arm_teleop_hold')

        # ===== CONFIG =====
        self.joints = ['linkk_1', 'link_2', 'link_3']
        self.min_theta = 0.0
        self.max_theta = math.radians(80.0)
        self.step = math.radians(5.0)   # 2° per cycle
        self.deadman = 7                 # RB

        # Button mapping: (increase, decrease)
        self.button_map = {
            'linkk_1': (3, 4),  # X, Y
            'link_2': (8, 9),   # LT, RT
            'link_3': (0, 1)    # A, B
        }

        # Current joint positions (initialize to 0)
        self.joint_positions = {j: 0.0 for j in self.joints}

        # Last button states
        self.last_buttons = []

        # ROS2 setup
        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_trajectory_controller/joint_trajectory',
            10
        )

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Timer for repeating while holding
        self.timer = self.create_timer(0.1, self.update_joints)  # 10 Hz
        self.current_joy = None

        self.get_logger().info("Button-hold Joystick Arm Teleop started")

    def joy_cb(self, msg: Joy):
        self.current_joy = msg
        if not self.last_buttons:
            self.last_buttons = [0]*len(msg.buttons)

    def update_joints(self):
        msg = self.current_joy
        if msg is None:
            return

        # Deadman switch
        if len(msg.buttons) <= self.deadman or msg.buttons[self.deadman] != 1:
            return

        updated = False
        for joint in self.joints:
            inc_btn, dec_btn = self.button_map[joint]

            # Increment if button held
            if inc_btn < len(msg.buttons) and msg.buttons[inc_btn]:
                self.joint_positions[joint] += self.step
                updated = True
            if dec_btn < len(msg.buttons) and msg.buttons[dec_btn]:
                self.joint_positions[joint] -= self.step
                updated = True

            # Clamp to limits
            self.joint_positions[joint] = max(
                self.min_theta, min(self.max_theta, self.joint_positions[joint])
            )

        if updated:
            traj = JointTrajectory()
            traj.joint_names = self.joints
            point = JointTrajectoryPoint()
            point.positions = [self.joint_positions[j] for j in self.joints]
            point.time_from_start.sec = 1
            traj.points.append(point)
            self.pub.publish(traj)

            angles_deg = [math.degrees(self.joint_positions[j]) for j in self.joints]
            self.get_logger().info(f"Joint angles: {angles_deg}")

def main():
    rclpy.init()
    node = JoyButtonArmTeleopHold()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JoyArmTrajectoryTeleop(Node):
    def __init__(self):
        super().__init__('joy_arm_traj_teleop')

        self.joints = ['linkk_1', 'link_2', 'link_3']
        self.min_theta = 0.0
        self.max_theta = math.radians(80.0)
        self.step = math.radians(5.0)
        self.deadman = 7   # RB

        self.button_map = {
            'linkk_1': (3, 4),
            'link_2': (8, 9),
            'link_3': (0, 1)
        }

        self.pos = {j: 0.0 for j in self.joints}

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_trajectory_controller/joint_trajectory',
            10
        )

        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        self.timer = self.create_timer(0.1, self.update)  # 10 Hz
        self.joy = None

        self.get_logger().info("✅ Trajectory joystick control ready")

    def joy_cb(self, msg):
        self.joy = msg

    def update(self):
        if self.joy is None:
            return

        # Deadman
        if self.joy.buttons[self.deadman] != 1:
            return

        updated = False

        for j in self.joints:
            inc, dec = self.button_map[j]

            if self.joy.buttons[inc]:
                self.pos[j] += self.step
                updated = True

            if self.joy.buttons[dec]:
                self.pos[j] -= self.step
                updated = True

            self.pos[j] = max(self.min_theta, min(self.max_theta, self.pos[j]))

        if not updated:
            return

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joints

        pt = JointTrajectoryPoint()
        pt.positions = [self.pos[j] for j in self.joints]
        pt.velocities = [0.3] * len(self.joints)
        pt.time_from_start.sec = 1   # >= 2x controller period

        traj.points.append(pt)
        self.pub.publish(traj)

        deg = [round(math.degrees(self.pos[j]), 1) for j in self.joints]
        self.get_logger().info(f"Joint deg: {deg}")

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmJoyController(Node):
    def __init__(self):
        super().__init__('arm_joy_controller')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_trajectory_controller/joint_trajectory',
            10
        )

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_cb,
            10
        )

        # Joint angles (DEGREES)
        self.joints = [0.0, 0.0, 0.0]

        self.step = 0.5
        self.MIN = 0.0
        self.MAX = 80.0

        # Button map
        self.ENABLE = 7

        self.J1_EXT = 0
        self.J1_RET = 1

        self.J2_EXT = 3
        self.J2_RET = 4

        self.J3_EXT = 8
        self.J3_RET = 9

    def joy_cb(self, msg):

        # Deadman check
        if not msg.buttons[self.ENABLE]:
            return

        updated = False

        # Joint 1
        if msg.buttons[self.J1_EXT]:
            self.joints[0] = min(self.MAX, self.joints[0] + self.step)
            updated = True
        elif msg.buttons[self.J1_RET]:
            self.joints[0] = max(self.MIN, self.joints[0] - self.step)
            updated = True

        # Joint 2
        if msg.buttons[self.J2_EXT]:
            self.joints[1] = min(self.MAX, self.joints[1] + self.step)
            updated = True
        elif msg.buttons[self.J2_RET]:
            self.joints[1] = max(self.MIN, self.joints[1] - self.step)
            updated = True

        # Joint 3
        if msg.buttons[self.J3_EXT]:
            self.joints[2] = min(self.MAX, self.joints[2] + self.step)
            updated = True
        elif msg.buttons[self.J3_RET]:
            self.joints[2] = max(self.MIN, self.joints[2] - self.step)
            updated = True

        if not updated:
            return

        traj = JointTrajectory()
        traj.joint_names = ['linkk_1', 'link_2', 'link_3']

        point = JointTrajectoryPoint()
        point.positions = self.joints  # DEGREES ✔️
        point.time_from_start.sec = 0

        traj.points.append(point)
        self.pub.publish(traj)

def main():
    rclpy.init()
    rclpy.spin(ArmJoyController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

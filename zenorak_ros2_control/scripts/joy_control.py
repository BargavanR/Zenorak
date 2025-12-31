#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import time

class ArmSequenceController(Node):
    def __init__(self):
        super().__init__('arm_sequence_controller')

        # ---- CONFIG ----
        self.joints = ['linkk_1', 'link_2', 'link_3']
        self.tolerance = 10.0  # degrees

        # Sequence in DEGREES
        self.sequence = [
            [0.0, 0.0, 0.0],
            [40.0, 40.0, 0.0],
            [50.0, 60.0, 80.0]
        ]

        self.current_step = 0
        self.current_feedback = None
        self.command_sent = False

        # ---- ROS ----
        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            '/arm_trajectory_controller/joint_trajectory',
            10
        )

        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_trajectory_controller/state',
            self.state_cb,
            10
        )

        self.timer = self.create_timer(0.2, self.update)  # 5 Hz

        self.get_logger().info("✅ Arm sequence controller started")

    def state_cb(self, msg):
        # Feedback already in DEGREES (as per your system)
        self.current_feedback = msg.actual.positions

    def within_tolerance(self, target, actual):
        for t, a in zip(target, actual):
            if abs(t - a) > self.tolerance:
                return False
        return True

    def send_command(self, target):
        traj = JointTrajectory()
        traj.joint_names = self.joints

        pt = JointTrajectoryPoint()
        pt.positions = target              # DEGREES
        pt.time_from_start.sec = 2          # smooth move

        traj.points = [pt]
        self.cmd_pub.publish(traj)

        self.get_logger().info(
            f"➡ Sent target (deg): {target}"
        )

    def update(self):
        if self.current_step >= len(self.sequence):
            self.get_logger().info("✅ Sequence complete (one cycle done)")
            return

        if self.current_feedback is None:
            return

        target = self.sequence[self.current_step]

        # Step 1: send command once
        if not self.command_sent:
            self.send_command(target)
            self.command_sent = True
            return

        # Step 2: wait for convergence
        if self.within_tolerance(target, self.current_feedback):
            self.get_logger().info(
                f"✔ Step {self.current_step} reached within ±{self.tolerance}°"
            )
            self.current_step += 1
            self.command_sent = False


def main():
    rclpy.init()
    node = ArmSequenceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

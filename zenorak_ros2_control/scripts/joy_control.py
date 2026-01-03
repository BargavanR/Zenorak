#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import Joy


class ArmSequenceController(Node):

    def __init__(self):
        super().__init__('arm_sequence_controller')

        # ---------------- CONFIG ----------------
        self.joints = ['linkk_1', 'link_2', 'link_3']
        self.tolerance = 10.0  # degrees

        # --------- SEQUENCES (INLINE) ----------
        self.EXCAVATION_SEQUENCE = [
            [0.0, 57.0, 40.0],
            [75.0, 57.0, 40.0],
            [0.0, 45.0, 45.0],
            [40.0, 20.0, 0.0],
            [80.0, 60.0, 70.0],
            [0.0, 60.0, 70.0],
        ]

        self.DUMP_SEQUENCE = [
            [0.0, 60.0, 70.0],
            [0.0, 80.0, 80.0],
            [0.0, 20.0, 0.0],
            [0.0, 45.0, 45.0],
        ]

        # ------------- STATE -------------------
        self.active_sequence = None
        self.sequence_name = ""
        self.current_step = 0
        self.command_sent = False
        self.current_feedback = None

        # ------------- ROS ---------------------
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

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_cb,
            10
        )

        self.timer = self.create_timer(0.2, self.update)  # 5 Hz

        self.get_logger().info("🎮 Arm Sequence Controller READY")

    # ---------------- CALLBACKS ----------------

    def state_cb(self, msg):
        self.current_feedback = msg.actual.positions

    def joy_cb(self, msg):
        # Button A (index 0) → Excavation
        if msg.buttons[0] == 1:
            self.start_sequence(self.EXCAVATION_SEQUENCE, "EXCAVATION")

        # Button B (index 1) → Dump
        elif msg.buttons[1] == 1:
            self.start_sequence(self.DUMP_SEQUENCE, "DUMP")

    # ---------------- LOGIC ----------------

    def start_sequence(self, sequence, name):
        if self.active_sequence is not None:
            return  # Ignore if already running

        self.active_sequence = sequence
        self.sequence_name = name
        self.current_step = 0
        self.command_sent = False

        self.get_logger().info(f"▶ {name} SEQUENCE STARTED")

    def within_tolerance(self, target, actual):
        for t, a in zip(target, actual):
            if abs(t - a) > self.tolerance:
                return False
        return True

    def send_command(self, target):
        traj = JointTrajectory()
        traj.joint_names = self.joints

        pt = JointTrajectoryPoint()
        pt.positions = target
        pt.time_from_start.sec = 2

        traj.points = [pt]
        self.cmd_pub.publish(traj)

        self.get_logger().info(
            f"➡ [{self.sequence_name}] Step {self.current_step}: {target}"
        )

    def update(self):
        if self.active_sequence is None:
            return

        if self.current_feedback is None:
            return

        if self.current_step >= len(self.active_sequence):
            self.get_logger().info(f"✅ {self.sequence_name} SEQUENCE COMPLETE")
            self.active_sequence = None
            self.sequence_name = ""
            return

        target = self.active_sequence[self.current_step]

        if not self.command_sent:
            self.send_command(target)
            self.command_sent = True
            return

        if self.within_tolerance(target, self.current_feedback):
            self.get_logger().info(
                f"✔ Step {self.current_step} reached (±{self.tolerance}°)"
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

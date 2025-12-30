#!/usr/bin/env python3
"""
Simple demo publisher for a one-shot JointTrajectory message.
Run after launching the controllers.
"""
import argparse
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def send_trajectory(joint_names, positions):
    rclpy.init()
    node = Node('demo_trajectory_pub')
    pub = node.create_publisher(JointTrajectory, '/arm_trajectory_controller/joint_trajectory', 10)

    msg = JointTrajectory()
    msg.joint_names = joint_names

    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.time_from_start.sec = 1  # default 1s for a smooth setpoint
    msg.points = [pt]

    pub.publish(msg)
    node.get_logger().info(f'Published JointTrajectory to {pub.topic_name}: {msg}')

    # Give RTPS a moment to transmit
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Publish a demo JointTrajectory')
    parser.add_argument('--joints', nargs='+', default=['linkk_1', 'link_2', 'link_3'],
                        help='Joint names for the trajectory (space-separated)')
    parser.add_argument('--positions', nargs='+', type=float, default=[0.0, 80.0, 0.0],
                        help='Positions (space-separated) matching the joints')
    args = parser.parse_args()

    if len(args.joints) != len(args.positions):
        raise SystemExit('Error: number of joint names must match number of positions')

    send_trajectory(args.joints, args.positions)

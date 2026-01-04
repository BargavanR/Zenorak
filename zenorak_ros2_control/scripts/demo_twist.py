#!/usr/bin/env python3
"""
Demo publisher that sends Twist messages to the differential drive controller topic.
It publishes at a small rate for the requested duration.
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def publish_twist(linear_x, angular_z, duration):
    rclpy.init()
    node = Node('demo_twist_pub')
    pub = node.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    end = time.time() + duration
    rate_hz = 10.0
    sleep_t = 1.0 / rate_hz

    t = Twist()
    t.linear.x = float(linear_x)
    t.angular.z = float(angular_z)

    node.get_logger().info(f'Publishing Twist to {pub.topic_name} for {duration}s: linear.x={linear_x} angular.z={angular_z}')

    while time.time() < end:
        pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(sleep_t)

    node.get_logger().info('Finished publishing demo twist')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Publish a demo Twist to the diff controller')
    parser.add_argument('--linear', type=float, default=0.2, help='Linear X velocity (m/s)')
    parser.add_argument('--angular', type=float, default=0.0, help='Angular Z velocity (rad/s)')
    parser.add_argument('--duration', type=float, default=2.0, help='Duration in seconds to publish')
    args = parser.parse_args()

    publish_twist(args.linear, args.angular, args.duration)

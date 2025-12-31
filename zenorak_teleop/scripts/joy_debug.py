#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyDebug(Node):
    def __init__(self):
        super().__init__('joy_debug')
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info("Joystick debug started")
        self.get_logger().info("Move sticks and press buttons to see indices")

    def joy_callback(self, msg: Joy):
        self.get_logger().info("----- JOY MESSAGE -----")

        # Axes
        for i, val in enumerate(msg.axes):
            self.get_logger().info(f"Axis[{i}]: {val:.3f}")

        # Buttons
        for i, val in enumerate(msg.buttons):
            self.get_logger().info(f"Button[{i}]: {val}")


def main():
    rclpy.init()
    node = JoyDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

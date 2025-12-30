#!/usr/bin/env python3
"""
Simple joystick-to-commands translator for Zenorak.
- Left joystick controls base when `enable_button` is pressed.
- `manip_enable_button` enables the manipulator buttons (Y/B/X/A) to be forwarded as commands.

Publishes:
- /diff_cont/cmd_vel_unstamped (geometry_msgs/Twist)
- /manipulator/button (std_msgs/String)  -- payload: 'Y','B','X','A' etc.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # Parameters - can be overridden from launch
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 0.5)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('enable_button', 6)           # left enable (default from user image)
        self.declare_parameter('manip_enable_button', 7)     # right enable
        # Button mapping: indexes -> label
        self.declare_parameter('manip_buttons', [3, 1, 2, 0])  # [Y, B, X, A] typical Xbox order
        # Optional second set (e.g. D-pad or other buttons)
        self.declare_parameter('manip_buttons_alt', [])

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.enable_button = self.get_parameter('enable_button').value
        self.manip_enable_button = self.get_parameter('manip_enable_button').value
        self.manip_buttons = self.get_parameter('manip_buttons').value
        self.manip_buttons_alt = self.get_parameter('manip_buttons_alt').value

        # publishers
        self.twist_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.manip_pub = self.create_publisher(String, '/manipulator/button', 10)

        # subscriber
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        self.get_logger().info('JoyTeleop started: left-stick -> base, buttons -> manipulator')

    def joy_cb(self, msg: Joy):
        # Read axes for left joystick (convention: axes[0]=left_x, axes[1]=left_y)
        ax_left_x = msg.axes[0] if len(msg.axes) > 0 else 0.0
        ax_left_y = msg.axes[1] if len(msg.axes) > 1 else 0.0

        # Check enable for base (motors)
        enable_base = False
        if len(msg.buttons) > self.enable_button:
            enable_base = bool(msg.buttons[self.enable_button])

        # If enabled, publish Twist using left stick
        if enable_base:
            lx = ax_left_y * self.linear_scale
            az = ax_left_x * self.angular_scale
            # deadzone
            if abs(lx) < self.deadzone:
                lx = 0.0
            if abs(az) < self.deadzone:
                az = 0.0
            t = Twist()
            t.linear.x = float(lx)
            t.angular.z = float(az)
            self.twist_pub.publish(t)
        else:
            # optionally, could publish zero twist when not enabled; keep previous behavior (no publish)
            pass

        # Manipulator enable (right enable)
        manip_enabled = False
        if len(msg.buttons) > self.manip_enable_button:
            manip_enabled = bool(msg.buttons[self.manip_enable_button])

        if manip_enabled:
            # primary set
            for idx, btn_index in enumerate(self.manip_buttons):
                if btn_index < len(msg.buttons) and msg.buttons[btn_index]:
                    label = ['Y', 'B', 'X', 'A']
                    lbl = label[idx] if idx < len(label) else str(btn_index)
                    out = String()
                    out.data = lbl
                    self.manip_pub.publish(out)
            # alternate set
            for btn_index in self.manip_buttons_alt:
                if btn_index < len(msg.buttons) and msg.buttons[btn_index]:
                    out = String()
                    out.data = f'BTN{btn_index}'
                    self.manip_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

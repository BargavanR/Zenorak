#!/usr/bin/env python3
# This script allows controlling a robot using keyboard keys (W, A, S, D + Enter)
# It publishes commands to a ROS 2 topic at 10Hz while a key is pressed.
# Once count reaches 50, it stops incrementing unless a new key is pressed.

import sys, termios, tty, threading
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class to create a ROS 2 node
from std_msgs.msg import String  # Standard ROS message type for sending strings
from rclpy.qos import QoSProfile, ReliabilityPolicy
import threading


class TeleopWASD(Node):
    """
    This ROS 2 Node reads keyboard input (W, A, S, D, Enter) and publishes
    corresponding movement commands as strings to the 'zenorak_teleop_cmd' topic.
    
    Key mappings:
      w -> f (forward)
      a -> l (left)
      s -> b (backward)
      d -> r (right)
      Enter -> s0 (stop/reset)
    
    The node sends incremental messages like 'f0', 'f1', ..., 'f50' at 10Hz
    while a key is pressed.
    """

    def __init__(self):
        # Initialize the ROS 2 node with a name 'zenorak_teleop'
        super().__init__('zenorak_teleop')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publisher to send String messages on the topic 'zenorak_teleop_cmd'
        self.pub = self.create_publisher(String, 'zenorak_teleop_cmd', 10)

        # Dictionary to map keyboard keys to command prefixes
        self.keymap = {'w': 'f', 'a': 'l', 's': 'b', 'd': 'r'}

        self.active_key = None  # Currently pressed key (None if no key)
        self.count = 120          # Counter for incremental message (0..50)
        self.lock = threading.Lock()  # Lock to safely handle shared variables

        # Start a separate thread to read keyboard input continuously
        self._run = True  # Control flag for the keyboard thread
        self.kthread = threading.Thread(target=self._read_keys, daemon=True)
        self.kthread.start()

        # Timer callback: calls self._tick() every 0.1 seconds (10Hz)
        self.timer = self.create_timer(1, self._tick)

    def _read_keys(self):
        """
        Reads single-character keyboard input in non-blocking mode.
        Updates active_key and count based on key pressed.
        """
        fd = sys.stdin.fileno()  # File descriptor for standard input
        old = termios.tcgetattr(fd)  # Save current terminal settings
        tty.setcbreak(fd)  # Switch terminal to raw mode (no Enter required)
        try:
            while self._run:  # Run until the node is destroyed
                c = sys.stdin.read(1)  # Read one character
                with self.lock:  # Lock to avoid race conditions
                    if c == '\n':  # Enter key pressed -> stop command
                        self.active_key = None
                        self.count = 0
                        msg = String()
                        msg.data = 's0'  # Stop/reset command
                        self.pub.publish(msg)
                        self.get_logger().info(f'Sent: {msg.data}')  # Log message
                    elif c in self.keymap:  # If W/A/S/D pressed
                        self.active_key = c  # Set active key
                        self.count = 120      # Reset counter for new key
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)  # Restore terminal settings

    def _tick(self):
        """
        Called every 0.1s by ROS 2 timer.
        Publishes incremental command messages while a key is active.
        """
        with self.lock:  # Lock to safely access active_key and count
            if self.active_key and self.active_key in self.keymap and self.count <= 120:
                msg = String()
                # Create message like 'f0', 'f1', ..., 'f50'
                msg.data = f"{self.keymap[self.active_key]}{self.count}"
                self.pub.publish(msg)  # Publish message to ROS 2 topic
                self.get_logger().info(f"Sent: {msg.data}")  # Log the message
                self.count += 1  # Increment count for next message

    def destroy_node(self):
        """
        Overrides Node.destroy_node to stop keyboard thread before destroying node.
        """
        self._run = False  # Stop keyboard thread
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize ROS 2, create the node, and spin it.
    """
    rclpy.init(args=args)  # Initialize ROS 2 Python client
    node = TeleopWASD()    # Create an instance of our teleop node
    try:
        rclpy.spin(node)  # Keep node alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow Ctrl+C to exit cleanly
    finally:
        node.destroy_node()  # Stop the node and cleanup
        rclpy.shutdown()     # Shutdown ROS 2


if __name__ == '__main__':
    main()  # Run the main function

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
import serial
import threading
import time

class ServoForceNode(Node):

    def __init__(self):
        super().__init__('servo_force_node')

        # Serial
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        time.sleep(2)  # Arduino reset

        # ROS interfaces
        self.create_subscription(Float32, '/servo_cmd', self.servo_cb, 10)
        self.servo_pub = self.create_publisher(Bool, '/servo_done', 10)
        self.ground_pub = self.create_publisher(String, '/ground_detected', 10)

        # Serial reader thread
        self.running = True
        threading.Thread(target=self.read_serial, daemon=True).start()

        self.get_logger().info("Servo + Force ROS node READY")

    def servo_cb(self, msg):
        angle = int(max(0, min(180, msg.data)))
        cmd = f"S {angle}\n"
        self.ser.write(cmd.encode())

    def read_serial(self):
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue

                if line == "OK":
                    msg = Bool()
                    msg.data = True
                    self.servo_pub.publish(msg)

                elif line == "ground":
                    msg = String()
                    msg.data = "ground"
                    self.ground_pub.publish(msg)

            except Exception:
                pass

    def destroy_node(self):
        self.running = False
        self.ser.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = ServoForceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

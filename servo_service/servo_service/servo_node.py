import rclpy
from rclpy.node import Node
from servo_service.srv import SetServoAngle
import Jetson.GPIO as GPIO
import time

SERVO_PIN = 33          # PWM pin (change if needed)
PWM_FREQ = 50           # 50Hz for servo

class ServoService(Node):

    def __init__(self):
        super().__init__('servo_service_node')

        # ROS service
        self.srv = self.create_service(
            SetServoAngle,
            'set_servo_angle',
            self.set_servo_callback
        )

        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
        self.pwm.start(0)

        self.get_logger().info("Servo service ready")

    def angle_to_duty(self, angle):
        # 0° → ~2.5%, 180° → ~12.5%
        return 2.5 + (angle / 180.0) * 10.0

    def set_servo_callback(self, request, response):
        angle = max(0, min(180, request.angle))
        duty = self.angle_to_duty(angle)

        self.get_logger().info(f"Moving servo to {angle}°")

        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)

        response.success = True
        response.message = f"Servo moved to {angle} degrees"
        return response

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

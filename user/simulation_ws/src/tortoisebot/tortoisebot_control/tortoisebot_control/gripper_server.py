import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO
import time


class GripperService(Node):
    def __init__(self):
        super().__init__("gripper_service")
        self.srv = self.create_service(
            SetBool, "gripper_command", self.gripper_callback
        )
        self.servo_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.p = GPIO.PWM(self.servo_pin, 50)
        self.p.start(2.5)
        self.get_logger().info("Gripper Service has been started")

    def gripper_callback(self, request, response):
        if request.data:  # if True, open the gripper
            self.p.ChangeDutyCycle(7.5)  # 90 degrees
            time.sleep(0.5)
            response.success = True
            response.message = "Gripper opened"
        else:  # if False, close the gripper
            self.p.ChangeDutyCycle(2.5)  # 0 degrees
            time.sleep(0.5)
            response.success = True
            response.message = "Gripper closed"
        return response


def main(args=None):
    rclpy.init(args=args)
    gripper_service = GripperService()
    rclpy.spin(gripper_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

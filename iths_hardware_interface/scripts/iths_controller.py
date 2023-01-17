
#!/usr/bin/python3

import rclpy
import time
import math

from rclpy.node import Node
from robot_serial import RobotSerial
from std_msgs.msg import String

# Number of steps to rotate one revolution
STEPS_PER_REVOLUTION = 512

class IthsController(Node):

    def __init__(self):
        super().__init__('iths_controller')

        self.left_arm_up = self.create_subscription(
            String, '/iths_left_arm/up', self.left_arm_up_callback, 100)
        self.left_arm_down = self.create_subscription(
            String, '/iths_left_arm/down', self.left_arm_down_callback, 100)
        self.right_arm_up = self.create_subscription(
            String, '/iths_right_arm/up', self.right_arm_up_callback, 100)
        self.left_arm_up = self.create_subscription(
            String, '/iths_right_arm/down', self.right_arm_down_callback, 100)
        self.left_arm_up = self.create_subscription(
            String, '/iths_head/left', self.head_left_callback, 100)
        self.left_arm_up = self.create_subscription(
            String, '/iths_head/right', self.head_right_callback, 100)

        self.robot = RobotSerial('/dev/ttyACM0')


    def left_arm_up_callback(self, msg):
        self.robot.move(1,1)
    def left_arm_down_callback(self, msg):
        self.robot.move(1,0)
    def right_arm_up_callback(self, msg):
        self.robot.move(2,0)
    def right_arm_down_callback(self, msg):
        self.robot.move(2,1)
    def head_left_callback(self, msg):
        self.robot.move(0,1)
    def head_right_callback(self, msg):
        self.robot.move(0,0)


def main(args=None):
    rclpy.init(args=args)
    controller = IthsController()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

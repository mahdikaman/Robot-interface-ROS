#!/usr/bin/env python3

import serial
import time

class RobotSerial:

    def __init__(self, device):
        self.arduino = serial.Serial(port=device, baudrate=115200, timeout=0.01)
        self.left_arm_position = 0

    def move(self, joint, direction):
        '''
        Writes a bytearray of size 3 to the arduino
        '''
        command = bytearray()

        control = 0b11000000
        direction = int(direction) << 2
        mirror = (direction << 3) | (int(joint) << 3)

        command.append(control | int(joint) | direction | mirror)

        self.arduino.write(command)


    def read_position(self):
        '''
        Reads data representing
        the joint position
        '''
        self.arduino.flush()
        data = self.arduino.readline()

        return data


if __name__ == "__main__":
    robot = RobotSerial('/dev/ttyACM0')

    while True:
        joint = input("Joint to move(0-2):")
        direction = input("Direction to move(0-1):")
        steps = input("Steps to move:")
        for i in range(0, int(steps)):
            robot.move(joint, direction)
            time.sleep(0.01)
            # print(robot.read_position())






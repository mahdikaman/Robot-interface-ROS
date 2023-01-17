#!/usr/bin/env python3

import rclpy
import time

from rclpy.action import ActionServer
from rclpy.node import Node
from iths.action import Move


class IthsActionServer(Node):

    def __init__(self):
        super().__init__('iths_action_server')
        self._action_server_right = ActionServer(
            self, Move, 'iths_left_arm', self.left_action_callback)
        self._action_server_left = ActionServer(
            self, Move, 'iths_right_arm', self.right_action_callback)
        self._action_server_torso = ActionServer(
            self, Move, 'iths_torso', self.torso_action_callback)

    def left_action_callback(self, goal_handle):
        return self.action_callback(goal_handle, "Left")

    def right_action_callback(self, goal_handle):
        return self.action_callback(goal_handle, "Right")

    def torso_action_callback(self, goal_handle):
        return self.action_callback(goal_handle, "Torso")

    def action_callback(self, goal_handle, name):
        self.get_logger().info('[' + name + ']: Executing goal...')

        # Get requested position
        position = goal_handle.request.position
        self.get_logger().info(
            '[' + name + ']: Position retrieved: ' + str(position))

        # Send feedback as requested position
        feedback_msg = Move.Feedback()
        feedback_msg.feedback = position
        goal_handle.publish_feedback(feedback_msg)

        # Set goal to succeed
        goal_handle.succeed()

        # Set status to true(done)
        result = Move.Result()
        result.status = True

        return result


def main(args=None):
    rclpy.init(args=args)

    iths_action_server = IthsActionServer()

    rclpy.spin(iths_action_server)


if __name__ == '__main__':
    main()

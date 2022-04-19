#!/usr/bin/env python

import os
import time

import rospy
from alienGo_deploy.msg import GamepadCommand


class JoystickRosInterface(object):
    def __init__(self, topic_name='/gamepad', gamepad_type='PS4'):
        from gamepad import gamepad, controllers
        if not gamepad.available():
            print('Please connect your gamepad...')
            while not gamepad.available():
                time.sleep(1.0)
        for available_gamepad_type in controllers.all_controllers:
            if gamepad_type.lower() == available_gamepad_type.lower():
                self.gamepad = getattr(controllers, available_gamepad_type)()
                self.gamepad_type = available_gamepad_type
                break
        else:
            raise RuntimeError('`{}` is not supported, all {}'.format(gamepad_type, controllers.all_controllers))
        self.cmd_pub = rospy.Publisher(topic_name, GamepadCommand, queue_size=1)
        self.gamepad.startBackgroundUpdates()
        self.flag = True
        print('Gamepad connected')

    @staticmethod
    def is_available():
        from gamepad import gamepad
        return gamepad.available()

    def get_cmd_and_publish(self):
        if self.gamepad.isConnected():
            msg = GamepadCommand()
            try:
                msg.left_x = self.gamepad.axis('LAS -X')
                msg.left_y = self.gamepad.axis('LAS -Y')
                msg.right_x = self.gamepad.axis('RAS -X')
                msg.right_y = self.gamepad.axis('RAS -Y')
                msg.LT = self.gamepad.axis('LT')
                msg.RT = self.gamepad.axis('RT')
                msg.A = self.gamepad.beenReleased('A')
                msg.B = self.gamepad.beenReleased('B')
                msg.X = self.gamepad.beenReleased('X')
                msg.Y = self.gamepad.beenReleased('Y')
                msg.LAS = self.gamepad.isPressed('LAS')
                msg.RAS = self.gamepad.isPressed('RAS')
                if not self.flag:
                    self.flag = True
                    print('recover')
            except ValueError as e:
                print(e)
                self.flag = False
            self.cmd_pub.publish(msg)
        else:
            raise RuntimeError('Gamepad Disconnected')

    def __del__(self):
        self.gamepad.disconnect()


if __name__ == '__main__':
    rospy.init_node('joystick_ros_interface')
    gamepad_type = rospy.get_param("gamepad_type", "xbox")
    joystick = JoystickRosInterface(gamepad_type=gamepad_type)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        joystick.get_cmd_and_publish()
        rate.sleep()
    del joystick
    os._exit(0)

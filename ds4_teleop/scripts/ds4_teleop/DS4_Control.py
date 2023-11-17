#!/usr/bin/env python

from statistics import mode
import rospy
from ds4_driver.msg import Feedback, Status
from math import *

RX = 0
RY = 1
LX = 2
LY = 3
R1 = 4
R2 = 5
R3 = 6
L1 = 7
L2 = 8
L3 = 9
UP = 10
DOWN = 11
LEFT = 12
RIGHT = 13
CROSS = 14
SQUARE = 15
TRIANGLE = 16
CIRCLE = 17
OPTIONS = 18
SHARE = 19
PS = 20
TRACKPAD = 21
TOUCH_X = 22
TOUCH_Y = 23
TOUCH_ACTIVE = 24
BATTERY = 25

ACC_Z = 26

class DS4_Control(object):
    def __init__(self, status_topic="status", feedback_topic="set_feedback",
                    read_freq=40):
        self._min_interval = 1.0/read_freq
        self._last_pub_time = rospy.Time()
        self._prev = Status()
        self._led = {
            "r": 0,
            "g": 0.5,
            "b": 0,
        }
        self.colors = {0: {"r": 0.0, "g": 0.5, "b": 0.0},
                        1: {"r": 0.0, "g": 0.0, "b": 0.5},
                        2: {"r": 0.5, "g": 0.1, "b": 0.0},
                        3: {"r": 0.5, "g": 0.5, "b": 0.0},
                        4: {"r": 0.5, "g": 0.0, "b": 0.5},
                        5: {"r": 0.5, "g": 0.5, "b": 0.5},
                        6: {"r": 0.5, "g": 0.0, "b": 0.0},
                      }
        self.joy_value = [0]*27

        self._pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
        rospy.Subscriber(status_topic, Status, self.joy_callback, queue_size=1)

        self.feedback = Feedback()
        self.feedback.set_led = True
        self.feedback.set_rumble = True

    def get_joy_data(self):
        return self.joy_value
    
    def set_joy_color(self, color):
        self._led = self.colors[color]

        self.feedback.led_r = 1#self._led["r"]
        self.feedback.led_g = 1#self._led["g"]
        self.feedback.led_b = 1#self._led["b"]
        # self.feedback.set_led_flash = True
        # self.feedback.led_flash_on = 1
        # self.feedback.rumble_small = 0.5

        self._pub_feedback.publish(self.feedback)

    def set_joy_rumble(self):
        pass

    def joy_callback(self, msg : Feedback):
        # now = rospy.Time.now()
        # if (now - self._last_pub_time).to_sec() < self._min_interval:
        #     return
        
        self.joy_value[RX] = -msg.axis_right_x
        self.joy_value[RY] = msg.axis_right_y
        self.joy_value[LX] = -msg.axis_left_x
        self.joy_value[LY] = msg.axis_left_y

        self.joy_value[R1] = msg.button_r1
        self.joy_value[R2] = msg.button_r2
        self.joy_value[R3] = msg.button_r3
        self.joy_value[L1] = msg.button_l1
        self.joy_value[L2] = msg.button_l2
        self.joy_value[L3] = msg.button_l3

        self.joy_value[UP] = msg.button_dpad_up
        self.joy_value[DOWN] = msg.button_dpad_down
        self.joy_value[LEFT] = msg.button_dpad_left
        self.joy_value[RIGHT] = msg.button_dpad_right

        self.joy_value[CROSS] = msg.button_cross
        self.joy_value[SQUARE] = msg.button_square
        self.joy_value[TRIANGLE] = msg.button_triangle
        self.joy_value[CIRCLE] = msg.button_circle

        self.joy_value[OPTIONS] = msg.button_options
        self.joy_value[SHARE] = msg.button_share
        self.joy_value[PS] = msg.button_ps
        self.joy_value[TRACKPAD] = msg.button_trackpad

        self.joy_value[BATTERY] = msg.battery_percentage

        self.joy_value[TOUCH_X] = msg.touch0.x
        self.joy_value[TOUCH_Y] = msg.touch0.y
        self.joy_value[TOUCH_ACTIVE] = msg.touch0.active

        self.joy_value[ACC_Z] = msg.imu.linear_acceleration.z

        # self._last_pub_time = now
 

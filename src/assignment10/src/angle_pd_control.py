#!/usr/bin/env python

import line as line
import steer_calibration as steer
import sys
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
import numpy as np
from sklearn import linear_model
from std_msgs.msg import UInt8
import math

import rospy
from nav_msgs.msg import Odometry
import utils

from assignment7_line_detection_pd_control.msg import Line


CONTROLLER_SKIP_RATE = 2

class AnglePdController:
    def __init__(self):
        # Get the angle/orientation of the car from the ceiling camera
        self.angle_sub = rospy.Subscriber("/localization/odom/11", Odometry, self.odom_callback, queue_size=100)

        # Publish direct the steering command
        self.pub_angle_steering = rospy.Publisher("steering", UInt8, queue_size=100)

        self.kp = 0.3
        self.kd = 0.7

        self.pd_error = 0
        self.derivative = 0
        self.control_variable = 0
        self.counter = 0
        self.projected_direction = 0.0
        self.speed_rpm = 100
        
    def callback(self, data):

        opp = detection_point-intersection_point
        adj = image_height - detection_row

        # Angle to the intersection point in degrees
        # projected_direction = 90-math.degrees(math.atan2(opp, adj))
        self.projected_direction += math.degrees(math.atan(float(opp)/adj))
        self.counter += 1

        if self.counter < CONTROLLER_SKIP_RATE:
            return

        averaged_direction = self.projected_direction/self.counter
        self.counter = 0
        self.projected_direction = 0
        
        last_pd_error = self.pd_error

        # When the car is positioned at the lane, an angle to the intersection approaches zero
        # Thus, the target ideal direction is 0 degrees
        self.pd_error = averaged_direction
        self.derivative = self.pd_error - last_pd_error
        
        control_variable = self.kp * self.pd_error + self.kd * self.derivative
        steering_command = steer.get_actuator_command(control_variable)
        
        print("Projected direction: {:.2f}, error: {:.2f}, derivative: {:.2f}, control var: {:.2f}, steering_command {}".format(
            averaged_direction, self.pd_error, self.derivative, control_variable, steering_command))
        
        info = ("Projected direction: {:.2f}, error: {:.2f}, derivative: {:.2f}, control var: {:.2f}, steering_command {}\n".format(
            averaged_direction, self.pd_error, self.derivative, control_variable, steering_command))
        
        with open('/home/oleksandra/Documents/catkin_ws_user/src/assignment7_line_detection_pd_control/src/pd_output.txt', 'a') as out:
            out.write(info)

        # Set only the wheel angle and use manual control publisher to start the car
        self.pub_steering.publish(steering_command)
 
 
# The global PD Controller
pd_controller = PDController()

def main(args):
    print("PD Lane Detection Controller Node launched")
    rospy.init_node('pd_controller', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + ": started!")

    steer.calibrate_steer()
    print("Steer commands mapped.")
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

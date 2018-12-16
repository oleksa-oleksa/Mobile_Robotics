#!/usr/bin/env python
'''
The Algorithm:
* Launch the lane_detection node and get the line equation with Ransac (assignment 6). 
* New feature added: custom message type Line for detected slope and intercept
* Calculate an error using PD controller from line parameters (new code created for assignment 7)
* Create a command for an actuator (code from assignment 6)
* Start a mobile robot with a manual speed control publisher from a terminal window
* Detect the new line position and calculate a new error and a new command and perform the next movement

Use rostopic pub /manual_control/speed std_msgs/Int16 to start a car with a fixed speed
'''

#imports from previous assignments
import line as line
import steer_calibration as steer
import rospy

#python imports
import sys
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn import linear_model
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import UInt8
from std_msgs.msg import String
import math

from assignment7_line_detection_pd_control.msg import Line
#from assignment7_line_detection_pd_control.msg import Drive
#from assignment7_line_detection_pd_control.srv import CarMovement

CONTROLLER_SKIP_RATE = 5

class PDController:
    def __init__(self):
        # Get the line parameters of detected line 
        self.pd_controller_sub = rospy.Subscriber("/line_parameters", Line, self.callback, queue_size=1)

        # Solution: publish direct the steering command
        self.pub_steering = rospy.Publisher("steering", UInt8, queue_size=100)

        # Parameters of PD Controller
        self.kp = 5 # kd = kp * 0.06 (Matlab PD Simulation for Faster and Robust response. )
        self.kd = 0.35
        # ===========================

        self.pd_error = 0
        self.derivative = 0
        self.control_variable = 0
        self.counter = 0
        self.projected_direction = 0.0
        self.speed_rpm = 100
        
    def callback(self, data):
        lane_slope = data.slope
        lane_intercept = data.intercept

        image_width = data.width
        image_height = data.height
        # Camera image row along which detection happens
        detection_row = image_height/2
        
        # Point along the detection line that's used as an anchor
        detection_point = image_width/2

        # Image column where detected lane line actually crosses detection line
        intersection_point = (detection_row - lane_intercept)/lane_slope

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


        # Set only the wheel angle and use manual control publisher to start the car
        self.pub_steering.publish(steering_command)
 
 
# The global PD Controller
pd_controller = PDController()

def main(args):
    print("PD Lane Detection Controller Node launched")
    rospy.init_node('pd_controller', anonymous=True)

    steer.calibrate_steer()
    print("Steer commands mapped.")
        
    rospy.loginfo(rospy.get_caller_id() + ": started!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

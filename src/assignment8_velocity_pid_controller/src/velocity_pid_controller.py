#!/usr/bin/env python
'''
Created by Oleksandra Baga

Write a PID-controller which makes sure that the model car is able to achieve and maintain a
certain velocity, which comes as an input and is specified in meters per second. Use the
pulse-sensor in the vehicle as a sensory feedback. The output of the controller shall be an
rpm-amount.

1 rpm: wheel rotates exactly once every minute

Rpm stands for rotations per minute and is used to quantify the speed at which an object spins, 
such as a motor or a centrifuge. Linear speed measures the actual distance traveled, 
often in meters per minute. Because a rotation always covers the same distance, 
you can convert from rpm to linear distance if you can find the distance per rotation. 
To do so, all you need is the diameter of the rotation.

'''

import sys
import rospy
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn import linear_model
from std_msgs.msg import Int32, Int16, UInt8, Float32
from std_msgs.msg import String
import math


CONTROLLER_SKIP_RATE = 5
PI = 3.14159265
WHEEL_DIAMETER = 0.07 # meter

class PIDController:
    def __init__(self):
        # Get the ticks im rpm
        self.pid_controller_sub = rospy.Subscriber("/ticks", Int16, self.callback, queue_size=1)

        # Solution: publish direct the steering command
        self.pub_steering = rospy.Publisher("speed", Int16, queue_size=100)
        
        self.kp = 0.2
        self.ki = 0
        self.kd = 0.9

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
        
        print("Projected direction: {}, error value: {}, error derivative: {}, control var: {}, steering_command {}".format(
            averaged_direction, self.pd_error, self.derivative, control_variable, steering_command))


        # Set only the wheel angle and use manual control publisher to start the car
        self.pub_steering.publish(steering_command)

 
# The global PID Controller
pid_controller = PIDController()

def main(args):
    print("PID Velocity Controller Node launched")
    rospy.init_node('pid_controller', anonymous=True)

        
    rospy.loginfo(rospy.get_caller_id() + ": started!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

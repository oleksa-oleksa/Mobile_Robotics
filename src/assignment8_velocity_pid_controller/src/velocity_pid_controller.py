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

PID:
We have the P controller for fast system response (improve rise time), 
I controller to correct the steady state error and 
D controller to dampen the system and reduce overshoot.

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
        self.pub_speed = rospy.Publisher("speed", Int16, queue_size=100)
        
        # Parameters of PID Controller
        # Matlab Simulation used for PID Controller with pidTuner()
        self.kp = 1.6  
        self.ki = 3.3 
        self.kd = 0.16 
        self.target_speed = 20 # mps
        # ===========================

        self.pid_error = 0
        self.integral = 0
        self.derivative = 0
        self.control_variable = 0
        self.counter = 0
        self.projected_speed = 0.0

        
    def callback(self, msg):
        # the current rpm got from encoder sensor
        current_rpm = msg.data

        # converting rpm to a linear speed
        current_speed = current_rpm * PI * WHEEL_DIAMETER
        
        self.projected_speed = self.projected_speed + current_speed
        
        self.counter += 1

        if self.counter < CONTROLLER_SKIP_RATE:
            return
        
        # get the average speed for PID control variable
        average_speed = self.current_speed/self.counter
        self.counter = 0
        self.current_speed = 0
        
        # PID CONTROLLER
        last_pid_error = self.pid_error

        self.pid_error = self.target_speed - average_speed
        self.integral = self.integral + self.pid_error
        self.derivative = self.pid_error - last_pid_error
        
        control_variable = self.kp * self.pd_error + self.ki * self.integral  + self.kd * self.derivative
        
        speed_command = control_variable / (PI * WHEEL_DIAMETER)
        
        print("Speed: {:.2f}, error: {:.2f}, integral: {:.2f}, derivative: {:.2f}, control var: {:.2f}, speed_command {}".format(
            average_speed, self.pd_error, self.integral, self.derivative, control_variable, speed_command))

        # Publish the speed
        self.pub_speed.publish(speed_command)

 
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

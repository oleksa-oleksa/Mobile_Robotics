#!/usr/bin/env python
'''
Ticks Subscriber

'''

import sys
import rospy
import cv2
import time
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn import linear_model
from std_msgs.msg import Int32, Int16, UInt8, Float32
from std_msgs.msg import String
import math


CONTROLLER_SKIP_RATE = 1
PI = 3.14159265
WHEEL_DIAMETER = 0.065 # meter


class PIDController:
    def __init__(self):
        # Get the ticks im rpm
        self.pid_controller_sub = rospy.Subscriber("/ticks", UInt8, self.callback, queue_size=1)
        
        # Parameters of PID Controller
        # Matlab Simulation used for PID Controller with pidTuner()
        self.kp = 0.9  
        self.ki = 0.003
        self.kd = 0.2
        self.target_speed = 150 # mps
        # ===========================

        self.pid_error = 0
        self.integral = 0
        self.derivative = 0
        self.control_variable = 0
        self.counter = 0
        self.projected_speed = 0.0
        self.ticks = []
        self.start = time.time()
        
        
    def callback(self, msg):
        # the current rpm got from encoder sensor
        current_rpm = msg.data
        
        stop = time.time()
        
        diff = stop - self.start
        
        if diff < 60:
            self.ticks.append(current_rpm)
            return
        else:
            self.start = time.time()
        
        
        all_ticks = sum(self.ticks) 
        print("RPM: {}".format(all_ticks))
        self.ticks = []
        return

        # converting rpm to a linear speed
        current_speed = current_rpm * PI * WHEEL_DIAMETER
        
        self.projected_speed = self.projected_speed + current_speed
        
        self.counter += 1

        if self.counter < CONTROLLER_SKIP_RATE:
            return
        
        # get the average speed for PID control variable
        average_speed = self.projected_speed/self.counter
        
        self.counter = 0
        self.current_speed = 0
        
        # PID CONTROLLER
        last_pid_error = self.pid_error

        self.pid_error = self.target_speed - average_speed
        self.integral = self.integral + self.pid_error
        self.derivative = self.pid_error - last_pid_error
        
        control_variable = self.kp * self.pid_error + self.ki * self.integral  + self.kd * self.derivative
        
        speed_command = control_variable / (PI * WHEEL_DIAMETER)

        print("RPM: {}".format(current_rpm))
        print("current linear speed: {}".format(current_speed))
        print("average speed: {}".format(average_speed))
        
        print("error: {:.2f}, integral: {:.2f}, derivative: {:.2f}, control var: {:.2f}, speed_command {}".format(
            self.pid_error, self.integral, self.derivative, control_variable, speed_command))
            
 
 
# The global PID Controller
pid_controller = PIDController()

def main(args):
    print("Ticks Node launched")
    rospy.init_node('ticks_subscriber', anonymous=True)

        
    rospy.loginfo(rospy.get_caller_id() + ": started!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

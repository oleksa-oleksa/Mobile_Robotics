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
import time

import ticks_subscriber as ticks
#from assignment8_velocity_pid_controller.src.ticks_subscriber import slope_speed,\
#    intercept_speed

global slope_distance, intercept_distance, slope_speed, intercept_speed

CONTROLLER_SKIP_RATE = 2
PI = 3.14159265
WHEEL_DIAMETER = 0.06 # meter

class Speedometer:
    
    def __init__(self, aggregation_period):
        self.aggregation_period = aggregation_period
        self.ticks = []
        self.ticks_sub = rospy.Subscriber("/ticks", UInt8, self._tick, queue_size=1)
        
    def get_speed(self):
        "Return average RMP reported by /ticks for the last aggregation_period secods" 
        n_ticks = len(self.ticks)
        
        if n_ticks == 0:
            return 0
        
        return float(sum(t[1] for t in self.ticks))/n_ticks
        
    
    def _tick(self, msg):
        now = rospy.get_time()
        reading = msg.data
        self.ticks.append((now, reading))
        
        too_old = lambda t: now-t[0] > self.aggregation_period
                
        self.ticks = [tick for tick in self.ticks if not too_old(tick)]
        
        
class PIDController:
    def __init__(self):
        # Get the ticks im rpm
        # self.ticks_pub = rospy.Subscriber("/ticks_per_minute", UInt8, self.callback, queue_size=1)
        self.ticks_sub = rospy.Subscriber("/ticks", UInt8, self.callback, queue_size=1)
        
        # Speedometer will report average RPM for the last two seconds
        #self.speedometer = Speedometer(2)
        
        #CONTROL_DELAY = 1
        # Launch self._callback every CONTROL_DELAY second
        #self.control_timer = rospy.Timer(CONTROL_DELAY, self._callback)
        
        # Solution: publish direct the steering command
        self.pub_speed = rospy.Publisher("speed", Int16, queue_size=100)
        
        # Parameters of PID Controller
        # Matlab Simulation used for PID Controller with pidTuner()
        self.kp = 0.5  
        self.ki = 0.9
        self.kd = 0.16 
        self.target_speed = 0.2 # mps

        # ===========================
        self.start = time.time()
        self.ticks = []
        self.pid_error = 0
        self.integral = 0
        self.derivative = 0
        self.control_variable = 0
        self.counter = 0
        self.projected_speed = 0.0

    def get_velocity(self):
        measured_rpm = np.array([[0.17, 150], [0.3, 200], [0.427, 250]]) 
        
        ransac = linear_model.RANSACRegressor()
                
        n = measured_rpm.shape[0]
        ransac.fit(measured_rpm[:,0].reshape((n, 1)), measured_rpm[:,1].reshape(n, 1))
        global slope_speed, intercept_speed
        intercept_speed = ransac.estimator_.intercept_
        slope_speed = ransac.estimator_.coef_
    
    def get_distance(self):
        # array of measured ticks to 
        measured_rpm = np.array([[209, 2.05], [227, 2.1], [241, 2.13]]) 
        
        ransac = linear_model.RANSACRegressor()
                
        n = measured_rpm.shape[0]
        ransac.fit(measured_rpm[:,0].reshape((n, 1)), measured_rpm[:,1].reshape(n, 1))
        global slope_distance, intercept_distance
        intercept_distance = ransac.estimator_.intercept_
        slope_distance = ransac.estimator_.coef_

    
    def callback(self, event):
        # the current rpm got from encoder sensor
        current_rpm = event.data
        # current_rps = self.speedometer().get_speed()
        
        if current_rpm == 0:
            return
        
        elif current_rpm != 0 and self.start == 0:       
            self.start = time.time()

        stop = time.time()
        
        diff = stop - self.start
        
        if diff <= 2:
            self.ticks.append(current_rpm)
            return
        
        self.get_distance() # wait period is 2 sec
        current_speed = (slope_distance * self.ticks + intercept_distance) / 2
        
        #current_speed = current_speed * PI * WHEEL_DIAMETER
                
        # PID CONTROLLER
        last_pid_error = self.pid_error

        self.pid_error = self.target_speed - current_speed
        self.integral = self.integral + self.pid_error
        self.derivative = self.pid_error - last_pid_error
        
        control_variable = self.kp * self.pid_error + self.ki * self.integral  + self.kd * self.derivative
        
        #speed_command = control_variable / (PI * WHEEL_DIAMETER)
        #self.target_speed(speed_command)
        self.get_velocity()
        speed_command = slope_speed * control_variable + intercept_speed
        
        message = "Target: {}, Speed: {}, error: {}, integral: {}, derivative: {}, control var: {}, speed_command {}"
        info = message.format(self.target_speed, current_speed, self.pid_error, self.integral, self.derivative, control_variable, speed_command)
        
        
        print(info)
        
        
        with open('/home/oleksandra/Documents/catkin_ws_user/src/assignment8_velocity_pid_controller/src/pid_output.txt', 'a') as out:
            out.write(info)
            
        
        self.start = time.time()

        # Publish the speed
        # Warning: Use this publisher only if you have tested the speed and sure what you do
        #self.pub_speed.publish(speed_command)

 
# The global PID Controller
pid_controller = PIDController()

def main(args):
    print("PID Velocity Controller Node launched")
    rospy.init_node('pid_controller', anonymous=True)       
    rospy.loginfo(rospy.get_caller_id() + ": started!")

    #ticks.calibrate_velocity()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

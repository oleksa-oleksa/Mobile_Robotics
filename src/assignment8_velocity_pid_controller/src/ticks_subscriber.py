#!/usr/bin/env python
'''
Ticks Subscriber

'''

import sys
import rospy
import cv2
import time
from math import sqrt
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn import linear_model
from std_msgs.msg import Int32, Int16, UInt8, Float32
from std_msgs.msg import String
import math
import steer_calibration as steer


slope_speed = None
intercept_speed = None
last_odom = None
        
class TicksMeausurer:
    def __init__(self, aggregation_period):
        self.aggregation_period = aggregation_period
        self.ticks = []
        self.ticks_sub = rospy.Subscriber("/ticks", UInt8, self.callback, queue_size=1)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callbackOdom, queue_size=100)
        self.pub_speed = rospy.Publisher("speed", Int16, queue_size=100)
        
        self.start = time.time()
        self.distance = 0.1 # 1 meter
        self.epsilon = 0.05   # allowed inaccuracy for distance calculation
            
    def get_speed(self):
        "Return average RMP reported by /ticks_per_minute for the last aggregation_period secods" 
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
        
    def callbackOdom(self, msg):
        global last_odom
        last_odom = msg

    
    def callback(self, event):
        current_rpm = event.data
        
        start_pos = last_odom.pose.pose.position
        current_distance = 0
         
        while not rospy.is_shutdown() and current_distance < (self.distance - self.epsilon):
            # collect ticks for a distance
            self.ticks.append(current_rpm)
            current_pos = last_odom.pose.pose.position
            current_distance = sqrt(
                (current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
        
        # stop the car
        self.pub_speed.publish(0)
        
        # check the distance
        current_pos = last_odom.pose.pose.position
        current_distance = sqrt((current_pos.x - start_pos.x)** 2 + (current_pos.y - start_pos.y)**2)
        
        rospy.loginfo("Actual travelled distance = %f", current_distance)
        
        stop = time.time()
        diff = stop - self.start
        
        all_ticks = sum(self.ticks) 
        print("Ticks: {}".format(all_ticks))
        
        info = ("self.distance : {:.2f}, current_distance: {:.2f}, all_ticks: {:.2f}, time: {:.2f}".format(
            self.distance, current_distance, all_ticks, diff))
        
        with open('/home/oleksandra/Documents/catkin_ws_user/src/assignment8_velocity_pid_controller/src/ticks_output.txt', 'a') as out:
            out.write(info)
        
        self.ticks = []
        return all_ticks
        

class TicksSubscriber:
    def __init__(self):
        # Get the ticks im rpm
        self.ticks_sub = rospy.Subscriber("/ticks", UInt8, self.callback, queue_size=1)
        self.ticks_pub = rospy.Publisher("ticks_per_minute", Int16, queue_size=100)
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
        return all_ticks


def calibrate_velocity():
    measured_rpm = np.array([[100, 496], [100, 541], [100, 473],
                             [150, 1232], [150, 1140], [150, 1160],
                             [200, 1806],[200, 1761],[200, 1620],
                             [250, 2305],[250, 2114],[250, 2052]]) 
    
    ransac = linear_model.RANSACRegressor()
    
    
    n = measured_rpm.shape[0]
    ransac.fit(measured_rpm[:,0].reshape((n, 1)), measured_rpm[:,1].reshape(n, 1))
    global slope_speed, intercept_speed
    intercept_speed = ransac.estimator_.intercept_
    slope_speed = ransac.estimator_.coef_
    

def get_ticks(command):
    command = slope_speed * command + intercept_speed
    return int(command)

 
def main(args):
    print("Ticks Node launched")
    rospy.init_node('ticks_subscriber', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + ": started!")
    
    pub_speed = rospy.Publisher("speed", Int16, queue_size=100)
    pub_steering = rospy.Publisher("steering", UInt8, queue_size=100)
        
    steer.calibrate_steer()
    steering_command = steer.get_actuator_command(0)
    print("stearing command for 0 grad: {}".format(steering_command))
    # 0 grad: go straight
    pub_steering.publish(steering_command)
    # np speed, remain steady before manual control start
    pub_speed.publish(0)
 
    measurer = TicksMeausurer(2)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

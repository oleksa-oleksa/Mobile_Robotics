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

slope_speed = None
intercept_speed = None

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
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

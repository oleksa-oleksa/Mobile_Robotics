#!/usr/bin/env python
'''
Use rostopic pub /manual_control/speed std_msgs/Int16 to start a car with a fixed speed
'''

#imports from previous assignments
import line as line
import steer_calibration as steer
import rospy

#python imports
import sys
import cv2
import vanishing_point_detection as vanish
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

from assignment12_time_and_precision.msg import LineSet, Line
from vanishing_point_detection import LaneDetection


CONTROLLER_SKIP_RATE = 2

def angle_to_guide_line(line, detection_row):
    lane_slope = line.slope
    lane_intercept = line.intercept

    image_width = line.width
    image_height = line.height
    
    # Point along the detection line that's used as an anchor
    detection_point = image_width/2
    detection_row = image_height - detection_row

    # Image column where detected lane line actually crosses detection line
    intersection_point = (detection_row - lane_intercept)/lane_slope

    opp = detection_point-intersection_point
    adj = image_height - detection_row
    return math.degrees(math.atan(float(opp)/adj))

def angle_to_line(line):
    lane_slope = line.slope
    lane_intercept = line.intercept

    image_width = line.width
    image_height = line.height
    # Camera image row along which detection happens
    detection_row = image_height/2
    
    # Point along the detection line that's used as an anchor
    detection_point = image_width/2

    # Image column where detected lane line actually crosses detection line
    intersection_point = (detection_row - lane_intercept)/lane_slope

    opp = detection_point-intersection_point
    adj = image_height - detection_row
    return math.degrees(math.atan(float(opp)/adj))


class PDController:
    def __init__(self):
        # Get the line parameters of detected line 
        self.pd_controller_sub = rospy.Subscriber("/line_parameters", LineSet, self.callback, queue_size=1)

        # Solution: publish direct the steering command
        self.pub_steering = rospy.Publisher("steering", UInt8, queue_size=100)

        # Parameters of PD Controller
        self.kp = 0.3 
        self.kd = 0.7
        # ===========================

        self.pd_error = 0
        self.derivative = 0
        self.control_variable = 0
        self.counter = 0
        self.projected_direction = 0.0
        self.speed_rpm = 100
        
    def callback(self, data):
        
        # Old approach:
        # Folow the one line
        #angle_to_closest_line = min(map(angle_to_line, data.lines), key=abs)
        
        #=============================
        # New approach:
        # Drive between two lines: side line and dashed line to stay inside the road 
        van_point_x, van_point_y = LaneDetection.vanishing_point(data.line_set[0], data.line_set[1])
        
        # guide line
        line1 = data[0]
        line2 = data[1]
        center_point_x = abs((line2[1][0] - line1[1][0])) / 2 + line1[1][0]
        guide_line = Line()
        guide_line.slope = LaneDetection.get_slope(center_point_x, data[0].height, van_point_x, van_point_y)
        guide_line.intercept = LaneDetection.get_intercept(van_point_x, van_point_y, guide_line.slope)
        guide_line.height = data[0].height
        guide_line.width = data[0].width
        
        detection_row = guide_line.height - van_point_y
        angle_to_guide_line = angle_to_guide_line(data[2], detection_row)

        self.projected_direction += angle_to_guide_line
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
        
        with open('/home/oleksandra/Documents/catkin_ws_user/src/assignment12_time_and_precision/src/pd_output.txt', 'a') as out:
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

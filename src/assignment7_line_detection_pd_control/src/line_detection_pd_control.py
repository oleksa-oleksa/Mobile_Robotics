#!/usr/bin/env python
'''
The Algorithm:
* Launch the lane_detection node and get the line equation with Ransac (assignment 6). 
* New feature added: custom message type Line for detected slope and intercept
* Launch the simple_drive_control node 
* Calculate an error using PD controller from line parameters (new code created for assignment 7)
* Create a command for an actuator (code from assignment 6)
* Move a mobile robot
* Detect the new line position and calculate a new error and a new command and perform the next movement
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
from std_msgs.msg import String
import math
from numpy import arctan

from assignment7_line_detection_pd_control.msg import Line, Drive
from assignment7_line_detection_pd_control.srv import CarMovement

CONTROLLER_SKIP_RATE = 10

class PDController:
    def __init__(self):
        self.pd_controller_sub = rospy.Subscriber("/line_parameters", Line, self.callback, queue_size=1)
        self.sub_info = rospy.Subscriber("simple_drive_control/info", String, callbackDrivingControl, queue_size=10)
        self.sub_drive = rospy.Service("pd_controller/drive_forward", CarMovement, callbackDriveForward)

        # All driving will simply be turning left or right at a constant speed
        self.drive_control = rospy.Publisher("simple_drive_control/forward", Drive, queue_size=10)

        self.pd_error = 0
        self.derivative = 0
        self.control_variable = 0
        self.kp = 0.005
        self.kd = 0.99
        self.counter = 0

        # PD will be activated when the drive command will be sent via ROS Service
        self.activated = True
        # Driving will be enabled after first movement
        self.enabled = False
        
        self.speed_rpm = 200
        
    def callback(self, data):
        self.counter += 1

        if counter < CONTROLLER_SKIP_RATE:
            return

        self.counter = 0

        lane_slope = data.slope
        lane_intercept = data.intercept

        image_width = data.width
        image_height = data.height

        # Camera image row along which detection happens
        detection_row = image_height/2
        
        # Point along the detection line that's used as an anchor
        detection_point = image_width/2

        # Image column where detected lane line actually crosses detection line
        intersection_point = (lane_intercept - detection_row)/lane_slope

        opp = detection_point-intersection_point
        adj = image_height - detection_line

        # Angle to the intersection point in degrees
        projected_direction = math.degrees(math.atan2(opp, adj))
        
        last_pd_error = self.pd_error

        # When the car is positioned at the lane, an angle to the intersection approaches zero
        # Thus, the target ideal direction is 0 degrees
        # Therefore, error term is a negative projected direction
        self.pd_error = -projected_direction
        self.derivative = self.pd_error - last_pd_error
        
        control_variable = self.kp * self.pd_error + self.kd * self.derivative
        steering_command = steer.get_actuator_command(angle)
        
        print("Projected direction: {}, error value: {}, error derivative: {}, control var: {}, steering_command {}".format(
            projected_direction, self.pd_error, self.derivative, control_variable, steering_command))

        drive_command = Drive()
        drive_command.distance = 0.0001
        drive_command.angle = steering_command
        drive_command.speed_rpm = 0 # enable drive in lab 
        # Without the driving speed, the car should just steer towards the detected line
        
        # Tell the car to turn the weels and drive forward
        self.drive_control.publish(drive_command)

        # # counter to eliminate the oscillation
        # self.counter += 1 
        
        # slope_line = data.slope
        # intercept_line = data.intercept
        
        # camera_height = data.height
        # camera_width = data.width
        
        # # we want two lines to intersect in the middle_of_the_screen = camera_width / 2

        # target_position = camera_width / 2
        
        # # Get the current position
        # # solving the equations:
        # # y1 = h / 2 (horizontal line in the middle of camera)
        # # y2 = slope * x + intercept
        # # y1 = y2 = (camera_height / 2 - intercept) / k
        # line_height = camera_height / 2
        # current_position = (line_height - intercept_line) / slope_line
        
        # last_pd_error = self.pd_error
        
        # #Calculate the error (P Controller)
        # self.pd_error = target_position - current_position

        # '''
        # Calculation error change (D part)
        # Negative values of derivative indicate an improvement (reduction) in the error signal. 
        # For example, if the last error was 20 and the current error is 10, the derivative will be -10. 
        # When these negative values are multiplied with a constant, Kd, and are added to the output of the loop, 
        # it can slow down the system when approaching the target.
        # '''
        # self.derivative = self.pd_error - last_pd_error

        # #Calculate the control variable
        # control_variable = self.kp * self.pd_error + self.kd * self.derivative
        
        # opposite_side = self.pd_error
        # adjacent_side = line_height
        # angle = math.degrees(arctan(opposite_side / adjacent_side))
        # print("Angle in grad: ", angle)

        # drive_msg = Drive()
        # drive_msg.distance = 0.0001
        # drive_msg.angle = 0
        # drive_msg.speed_rpm = 0
        
        # # Speed will be enabled after PD will be tested in Lab
        # if not self.enabled:
        #     drive_msg.speed_rpm = 0
        # elif self.enabled:
        #     drive_msg.speed_rpm = self.speed_rpm
        
        # # move a car
        # # positive control_variable: turn left with positive angle value

        # print("Control variable: ", control_variable)
        # if self.counter > 20:
    
        #     if control_variable > 0:
        #         actuator_command = steer.get_actuator_command(angle)
        #         drive_msg.angle = actuator_command
        #         print("Actuator command: ", drive_msg.angle)
        #         self.pub_forward_right.publish(drive_msg)
        #         self.pub_actuator_commands.publish(actuator_command)
        #         self.counter = 0
    
        #     # negative control_variable: turn right with negative angle value
        #     elif control_variable < 0:
        #         actuator_command = steer.get_actuator_command(-angle)
        #         drive_msg.angle = actuator_command
        #         print("Actuator command: ", drive_msg.angle)
        #         self.pub_forward_left.publish(drive_msg)
        #         self.pub_actuator_commands.publish(actuator_command)
        #         self.counter = 0
                
        #     elif control_variable == 0:
        #         actuator_command = steer.get_actuator_command(0)
        #         drive_msg.angle = actuator_command                
        #         print("Actuator command: ", drive_msg.angle)
        #         self.pub_forward_straight.publish(drive_msg)
        #         self.pub_actuator_commands.publish(actuator_command)
        #         self.counter = 0
        #         self.enabled = False
        #         self.activated = False
            
    
def callbackDrivingControl(msg):
    last_driving_control_info = msg.data

def callbackDriveForward(req):
    rospy.loginfo(rospy.get_caller_id())
    print("PD status is started...")

    # PD-controller will start to work and move a car
    pd_controller.activated = True
    pd_controller.enabled = False # set to true after steer testing without driving

pd_controller = PDController()


def main(args):
    print("PD Node launched")
    rospy.init_node('pd_controller', anonymous=True)

    steer.calibrate_steer()
    
    # place the car in initial position and start service
    # in terminal: rosservice call /pd_controller/drive_start
    sub_drive_start = rospy.Service("pd_controller/drive_start", CarMovement, callbackDriveForward)
    print("rosservice call /pd_controller/drive_start to start PD controller")
    
    rospy.loginfo(rospy.get_caller_id() + ": started!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)

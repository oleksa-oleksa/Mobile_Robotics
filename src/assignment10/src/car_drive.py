#!/usr/bin/env python
import rospy
import numpy as np
import utils
from nav_msgs.msg import Odometry
from assignment10.msg import Coordinate
import math
import steer_calibration as steer
from std_msgs.msg import UInt8

CONTROLLER_SKIP_RATE = 45

class CarDrive:
    def __init__(self):
        self.sub_odom = rospy.Subscriber("/expected_coordinates", Coordinate, self.callback, queue_size=1)
        self.pub_steering = rospy.Publisher("steering", UInt8, queue_size=100)

        self.counter = 0

        self.derivative = 0
        self.control_variable = 0
        self.kp = 0.2
        self.kd = 0.9
        self.counter = 0
        self.projected_angle = 0.0
        self.average_angle = 0.0
        self.last_average = 0.0

    def callback(self, data):
        target_angle = math.degrees(np.arctan(data.expected_y / data.expected_x))
        current_angle = math.degrees(data.current_angle)
        print("Cardrive Target angle: ", target_angle, ", Current Angle: ", current_angle)

        self.projected_angle += target_angle
        self.counter += 1

        if self.counter < CONTROLLER_SKIP_RATE:
            return
        self.average_angle = self.projected_angle / self.counter
        self.derivative = (self.average_angle - self.last_average)
        self.last_average = self.derivative
        #self.average_angle = target_angle
        #self.derivative = (current_angle - target_angle) % 90

        #print("Cardrive derivative", self.derivative)
        control_variable = self.kp * self.average_angle + self.kd * self.derivative
        self.projected_angle = 0
        self.counter = 0

        steering_command = steer.get_actuator_command(control_variable)
        print("Cardrive derivative", self.derivative, ", Steering command: ", steering_command)
        self.pub_steering.publish(steering_command)


def main():
    print("CarDrive Node launched")
    rospy.init_node('car_drive', anonymous=True)
    car_drive = CarDrive()
    steer.calibrate_steer()

    rospy.loginfo(rospy.get_caller_id() + ": started!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()


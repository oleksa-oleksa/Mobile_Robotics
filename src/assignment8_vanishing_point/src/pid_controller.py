#!/usr/bin/env python

import rospy
import numpy as np
from assignment8.msg import Line, LineArray, MsgDrive
import math
import steer_calibration as steer
import sys

class PIDController:
    def __init__(self):
        print('Hola PID')
        self.kp = 0.3
        self.ki = 0
        self.kd = 0.7
        self.pid_error = 0
        self.projected_direction = 0.0
        self.counter = 0
        self.CONTROLLER_SKIP_RATE = 2
        self.integral = 0
        self.derivative = 0
        self.min_speed = 180
        self.max_speed = 250


        self.pid_sub = rospy.Subscriber("line_parameters", LineArray, self.callback, queue_size=1)
        self.pub_drive = rospy.Publisher("pid/drive", MsgDrive, queue_size=100)

    def callback(self, lines):
        guide_line = lines.lines[2]

        detection_row = guide_line.height / 2
        detection_point = guide_line.width / 2

        intersection_point = (detection_row - guide_line.intercept) / guide_line.slope

        opp = detection_point - intersection_point
        adj = guide_line.height - detection_row
        self.projected_direction += math.degrees(math.atan(float(opp) / adj))

        self.counter += 1

        if self.counter < self.CONTROLLER_SKIP_RATE:
            return

        averaged_direction = self.projected_direction / self.counter
        self.counter = 0
        self.projected_direction = 0

        last_pd_error = self.pid_error

        self.pid_error = averaged_direction
        self.integral = self.integral + self.pid_error
        self.derivative = self.pid_error - last_pd_error

        control_variable = self.kp * self.pid_error + self.ki * self.integral + self.kd * self.derivative
        steering_command = steer.get_actuator_command(control_variable)

        msgDrive = MsgDrive()
        msgDrive.angle = steering_command
        if steering_command > 75 and steering_command < 105:
            msgDrive.speed = self.max_speed
        else:
            msgDrive.speed = self.min_speed

        self.pub_drive.publish(msgDrive)


def main(args):
    print("PID Lane Detection Controller Node launched")
    rospy.init_node('pid_controller', anonymous=True)

    pid_controller = PIDController() #Prueba

    rospy.loginfo(rospy.get_caller_id() + ": started!")

    steer.calibrate_steer()
    print("Steer commands mapped.")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)





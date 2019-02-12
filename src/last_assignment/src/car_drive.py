#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, Int16, String
from last_assignment.msg import Coordinate
import utils, steer_calibration as steer
import numpy as np

CONTROLLER_SKIP_RATE = 2


class CarDrive:
    def __init__(self):
        self.inner_track = utils.calculate_shape(26)
        self.outer_track = utils.calculate_shape(29)
        self.speed_pub = rospy.Publisher("manual_control/speed", Int16, queue_size=100)
        self.lane_switcher_sub = rospy.Subscriber("/lane_switcher", Coordinate, self.lane_switch_callback, queue_size=1)
        self.pub_steering = rospy.Publisher("steering", UInt8, queue_size=100)
        self.dist_to_projection = 40
        self.dist_from_lane = 20
        self.val = 65
        self.kp = 10
        self.kd = 2
        self.pd_error = 0
        self.derivative = 0
        self.control_variable = 0
        self.counter = 0
        self.projected_direction = 0.0
        self.speed_rpm = 150
        self.previous_angle = 0
        self.all_angles = []

        f = open('/home/adripinto/catkin_ws_user/src/last_assignment/src/log.txt', 'r+')
        f.truncate(0)
        #f.write('current_x, current_y, closest_x, closest_y, expected_x, expected_y, distance\n')
        #f.close()

    def lane_switch_callback(self, car_position):
        #if lane == 'stop':
        #    self.speed_pub.publish(0)

        #car_position = rospy.wait_for_message("/expected_coordinates", Coordinate)
        # print("car_drive: &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
        # print("car_drive: car_position:", car_position.current_x, car_position.current_y)
        # print("car_drive: car_position_lane:", car_position.inner_x, car_position.inner_y)
        # print("car_drive: car_lane_proj:", car_position.inner_proj_x, car_position.inner_proj_y)
        # print("car_drive: lane: ", lane.data)

        if car_position.lane == 'inner':
            proj_point = utils.perpendicular_point(
                car_position.inner_proj_x * 100,
                car_position.inner_proj_y * 100,
                car_position.inner_x * 100,
                car_position.inner_y * 100,
                self.dist_to_projection
            )

            #print("car_drive: inner: proj_point:", proj_point)

            target_point_lane, distance = utils.get_closest_point(
                proj_point[0],
                proj_point[1],
                self.inner_track
            )

            #print("car_drive: inner: target_point_lane:", target_point_lane)

            target_point = utils.get_point_to_distance(
                self.inner_track,
                (proj_point[0], proj_point[1]),
                target_point_lane,
                self.dist_from_lane
            )
        else:
            proj_point = utils.perpendicular_point(
                car_position.outer_proj_x * 100,
                car_position.outer_proj_y * 100,
                car_position.outer_x * 100,
                car_position.outer_y * 100,
                self.dist_to_projection
            )

            #print("car_drive: outer: proj_point:", proj_point)

            target_point_lane, distance = utils.get_closest_point(
                proj_point[0],
                proj_point[1],
                self.inner_track
            )

            #print("car_drive: outer: target_point_lane:", proj_point)

            target_point = utils.get_point_to_distance(
                self.inner_track,
                (proj_point[0], proj_point[1]),
                target_point_lane,
                -self.dist_from_lane
            )

        #print("car_drive: target_point: ", target_point)
        adj = np.linalg.norm(
            np.array((car_position.current_y, car_position.current_x)) - np.array((target_point[0], target_point[1]))
        )

        #Logging the position sent by the camera (current_x, current_y) and the calculated position (calc_x, calc_y)
        info = ("{:.2f}, {:.2f}, {:.2f}, {:.2f}\n".format(
           target_point[0],
           target_point[1],
           target_point_lane[0],
           target_point_lane[1],
        ))
        with open('/home/adripinto/catkin_ws_user/src/last_assignment/src/log.txt', 'a') as out:
           out.write(info)

        ###############################################################################################################
        target_point = np.array(target_point)
        car_point = np.array((car_position.current_y * 100, car_position.current_x * 100))
        middle_point = target_point - car_point
        unit_point = np.array((0, 1))
        # print(closest_point_vector)
        # print((closest_point_vector))
        # print(vector)
        closest_angle = np.arccos(
            np.dot(unit_point, middle_point) / (np.linalg.norm(unit_point) * np.linalg.norm(middle_point))
        )

        print("car_drive: ###################################################")
        print("car_drive: car_position: current_angle:", np.rad2deg(car_position.current_angle))
        print("car_drive: closest_angle:", np.rad2deg(closest_angle))
        print("car_drive: middle_point:", middle_point)

        # # print(yaw,closest_point_error)
        if middle_point[0] >= 0:
            closest_point_error = closest_angle * -1# / np.pi
            theta = closest_point_error + car_position.current_angle
        else:
            closest_point_error = closest_angle
            theta = closest_point_error - car_position.current_angle
        if theta > np.pi:
            theta -= 2 * np.pi
        elif theta < -1 * np.pi:
            theta += 2 * np.pi

        print("car_drive: closest_point_error:", closest_point_error)
        print("car_drive: thetha:", np.rad2deg(theta))
        self.previous_angle = theta

        steering_angle = self.kp * theta + self.kd * (theta - self.previous_angle)
        steering_angle = np.rad2deg(steering_angle) + self.val # / 180

        if steering_angle > 180:
            steering_angle -= 180
        if steering_angle > 270:
            steering_angle -= 270
        print("car_drive: steering_angle:", steering_angle)

        self.all_angles.append(steering_angle)
        # print(steering_angle)
        self.counter += 1
        if self.counter >= CONTROLLER_SKIP_RATE:
            mean_error = np.median(self.all_angles)
            current_angle = mean_error
            print("car_drive: mean_error:", current_angle)
            if current_angle > 180:
                current_angle = 180
            if current_angle < 0:
                current_angle = 0
            self.counter = 0
            self.all_angles = []
            print("car_drive: current_angle:", current_angle)
            steering_command = steer.get_actuator_command(current_angle)
            print("car_drive: steering_command:", steering_command)

            self.pub_steering.publish(current_angle)

    ###############################################################################################################
        # print("car_drive: adjacent: ", adj)
        #
        # #theta = np.arctan((self.dist_from_lane/adj))
        #
        # print("car_drive: theta: ", np.rad2deg(theta))
        # theta = np.rad2deg(theta)
        #
        # ###############################################################################################################
        # #self.projected_direction += np.arctan((self.dist_from_lane/adj))
        # self.projected_direction += theta
        # self.counter += 1
        #
        # if self.counter < CONTROLLER_SKIP_RATE:
        #     return
        #
        # averaged_direction = self.projected_direction / self.counter
        # self.counter = 0
        # self.projected_direction = 0
        #
        # last_pd_error = self.pd_error
        # self.pd_error = averaged_direction
        # self.derivative = self.pd_error - last_pd_error
        # control_variable = self.kp * self.pd_error + self.kd * self.derivative
        #
        # print("car_drive: control_variable:", control_variable)
        # steering_command = steer.get_actuator_command(control_variable)
        #
        # self.pub_steering.publish(steering_command)



def main():
    print("CarDrive Node launched")
    rospy.init_node('car_drive', anonymous=True)
    car_drive = CarDrive()
    rospy.loginfo(rospy.get_caller_id() + ": started!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()

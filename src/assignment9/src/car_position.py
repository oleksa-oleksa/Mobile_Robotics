#!/usr/bin/env python
import rospy
import utils2
from nav_msgs.msg import Odometry

class CarPosition:
    def __init__(self):
        self.width, self.x1, self.y1, self.r1, self.x2, self.y2, self.r2, self.x3, self.x4 = utils2.calculate_shape()

        self.sub_odom = rospy.Subscriber("/localization/odom/11", Odometry, self.odom_callback, queue_size=100)
        f = open('/home/adripinto/catkin_ws_user/src/assignment9/src/final_coords.txt', 'r+')
        f.truncate(0)
        f.write('current_x, current_y, calc_x, calc_y, distance\n')
        f.close()

    def odom_callback(self, data):
        current_x = data.pose.pose.position.x
        current_y = data.pose.pose.position.y

        coords, distance = utils2.get_closest_point(
            current_x,
            current_y,
            self.width,
            self.x1,
            self.y1,
            self.r1,
            self.x2,
            self.y2,
            self.r2,
            self.x3,
            self.x4
        )


        #Logging the position sent by the camera (current_x, current_y) and the calculated position (calc_x, calc_y)
        info = ("{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}\n".format(current_y, current_x, coords[0], coords[1]    , distance))
        with open('/home/adripinto/catkin_ws_user/src/assignment9/src/final_coords.txt', 'a') as out:
            out.write(info)


def main():
    print("CarPosition Node launched")
    rospy.init_node('car_position', anonymous=True)
    car_position = CarPosition()
    rospy.loginfo(rospy.get_caller_id() + ": started!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
#!/usr/bin/env python
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from assignment8.msg import Line, LineArray

import sys


class LaneDetection:

    def __init__(self):
        print("Hola mundo")
        self.camera_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        self.line_pub = rospy.Publisher("/line_parameters", LineArray, queue_size=1)
        self.bridge = CvBridge()

    @staticmethod
    def line_segments(img):
        (_, contours, _) = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

        return contours[:2]

    @staticmethod
    def ransac_method(contour):
        ransac = RANSACRegressor()

        x = []
        y = []
        for val in contour:
            x.append([val[0][1]])
            y.append([val[0][0]])

        ransac.fit(x, y)
        b = ransac.estimator_.intercept_
        m = ransac.estimator_.coef_

        return m, b

    @staticmethod
    def end_start_points(m, b, width):
        x1 = 0
        x2 = width
        y1 = (x1 * m + b)
        y2 = (x2 * m + b)
        print("points: y1 = %f and x1 = %f , y2 = %f and x2 = %f" % (y1, x1, y2, x2))
        return ((y1, x1), (y2, x2))

    @staticmethod
    def vanishing_point(line1, line2):
        x1 = line1[0][1]
        x2 = line1[1][1]
        y1 = line1[0][0]
        y2 = line1[1][0]

        x3 = line2[0][1]
        x4 = line2[1][1]
        y3 = line2[0][0]
        y4 = line2[1][0]

        # Find the intersection point
        x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / (
                    (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
        y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / (
                    (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))

        return x, y

    @staticmethod
    def get_slope(x1, y1, x2, y2):
        slope = (y1 - y2) / (x1 - x2)
        return slope

    @staticmethod
    def get_intercept(x1, y1, slope):
        intercept = y1 - slope * x1
        return intercept

    @classmethod
    def get_guide_line(cls, img, line1, line2):
        van_point_x, van_point_y = cls.vanishing_point(line1, line2)
        print('Vanishing point: x = %f and y = %f' % (van_point_x, van_point_y))
        center_point_x = abs((line2[1][0] - line1[1][0])) / 2 + line1[1][0]
        print('Center point x = %f, y = %f' % (center_point_x, img.shape[0]))
        cv2.circle(img, (van_point_y, van_point_x), 10, (0, 255, 0))
        cv2.circle(img, (center_point_x, img.shape[0]), 10, (0, 255, 0))
        # cv2.imshow('img', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        guide_line = Line()
        guide_line.slope = cls.get_slope(center_point_x, img.shape[0], van_point_x, van_point_y)
        guide_line.intercept = cls.get_intercept(van_point_x, van_point_y, guide_line.slope)
        guide_line.height = img.shape[0]
        guide_line.width = img.shape[1]

        return guide_line



    def callback(self, data):
        print("New image received")
        try:
            # Camera
            img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        # pub_img_lines = rospy.Publisher("/image_processing/bin_img", Image, queue_size=1)

        # Crop 20% of the image along the y axis
        y_end = np.shape(img)[0]
        y_start = (np.shape(img)[0] * 0.2)
        img = img[int(y_start): int(y_end), :]

        # Convert RGB to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of color in HSV
        lower = np.array([0, 40, 150])
        upper = np.array([18, 80, 255])

        # Threshold the HSV image to get only the lines colors
        mask = cv2.inRange(hsv, lower, upper)

        segments = self.line_segments(mask)

        line_array = LineArray()
        counter = 1
        for segment in segments:
            m, b = self.ransac_method(segment)
            print("Equation line %d: y1 = %fx + %f" % (counter, m, b))
            # creating the custom message
            line_parameters = Line()
            line_parameters.slope = m
            line_parameters.intercept = b
            line_parameters.height = img.shape[0]
            line_parameters.width = img.shape[1]
            line_array.lines.append(line_parameters)
            counter += 1

        line1 = self.end_start_points(
            line_array.lines[0].slope,
            line_array.lines[0].intercept,
            line_array.lines[0].width

        )
        line2 = self.end_start_points(
            line_array.lines[1].slope,
            line_array.lines[1].intercept,
            line_array.lines[1].width
        )
        guide_line = self.get_guide_line(img, line1, line2)

        print("Guide line slope = %f and Intercept = %f" % (guide_line.slope, guide_line.intercept))
        line_array.lines.append(guide_line)

        self.line_pub.publish(line_array)


def main(args):
    rospy.init_node('lane_detector', anonymous=True)
    lane_detector = LaneDetection()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

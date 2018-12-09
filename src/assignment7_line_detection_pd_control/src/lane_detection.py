#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
import line as ld
import sys
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn import linear_model
from assignment7_line_detection_pd_control.msg import Line


class lane_detection:
    def __init__(self):
        self.lane_detection_pub = rospy.Publisher("/image_processing/lane", Image, queue_size=10)
        self.line_parameters_pub = rospy.Publisher("/line_parameters", Line, queue_size=10)
        
        self.bridge = CvBridge()
        self.lane_detection_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
 
    def callback(self, data):
        
        try:
            img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        
        
        # Crop 20% of the image along the y axis
        y_end = np.shape(img)[0]
        y_start = (np.shape(img)[0] * 0.2)
        img = img[int(y_start): int(y_end), :]
    
        # Convert RGB to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # define range of color in HSV
        sensitivity = 100
        lower = np.array([0, 0, 255 - sensitivity])
        upper = np.array([255, sensitivity, 255])
    
        # Threshold the HSV image to get only the lines colors
        mask = cv2.inRange(hsv, lower, upper)
        
        # Fill the 2/5 of picture with a black color
        h, w = mask.shape
        cv2.rectangle(mask, (0,0), (w, 2*h/5), 0, cv2.FILLED)
    
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=mask)
               
        seg = ld.line_segments(mask)
    
        intercept, slope = ld.ransac_method(seg)
        print("Equation line: y1 = %fx + %f" % (slope, intercept))
        
        #creating the custom message
        line_parameters = Line()
        line_parameters.slope = slope
        line_parameters.intercept = intercept
        line_parameters.height = img.shape[0]
        line_parameters.width = img.shape[1]
        
        line = ld.end_start_points(slope, intercept, img.shape[1])
        
        ransac_lines = ld.show_lines(img, line)
            
        try:
            # Test Unit
            #img = self.bridge.cv2_to_imgmsg(img, "rgb8")

            # Ransac 
            img = self.bridge.cv2_to_imgmsg(ransac_lines, "rgb8")
            self.lane_detection_pub.publish(img)
            self.line_parameters_pub(line_parameters)
            
        except CvBridgeError as e:
            print(e)
    
def main(args):
    rospy.init_node('lane_detector', anonymous=True)
    lane_detector = lane_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()    
    

if __name__ == '__main__':
    main(sys.argv)

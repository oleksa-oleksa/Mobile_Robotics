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
from assignment7_line_detection_smallest_angle.msg import Line, LineSet

#from assignment7_line_detection_pd_control.msg import Drive
#from assignment7_line_detection_pd_control.srv import CarMovement


class lane_detection:
    def __init__(self):
        self.lane_detection_pub = rospy.Publisher("/image_processing/lane", Image, queue_size=10)
        self.bw_detection_pub = rospy.Publisher("/image_processing/bw_contours", Image, queue_size=10)
        self.line_parameters_pub = rospy.Publisher("/line_parameters", LineSet, queue_size=10)
        
        self.bridge = CvBridge()
        self.lane_detection_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
 
    def callback(self, data):
        
        try:
            # Camera 
            img = self.bridge.imgmsg_to_cv2(data, "rgb8") 
        except CvBridgeError as e:
            print(e)
        
        im_h, im_w, _ = img.shape
     
        # Convert RGB to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
         
        # define range of color in HSV
        sensitivity = 100
        lower = np.array([0, 0, 255 - sensitivity])
        upper = np.array([255, sensitivity, 255])
     
        # Threshold the HSV image to get only the lines colors
        mask = cv2.inRange(hsv, lower, upper)
        mask_pub = mask
        # Fill the 2/5 of picture with a black color
        h, w = mask.shape
        cv2.rectangle(mask, (0,0), (w, 3*h/4), 0, cv2.FILLED)
        cv2.rectangle(mask, (0,0), (w/4, h), 0, cv2.FILLED)
        cv2.rectangle(mask, (w/2,0), (w/2, h), 0, cv2.FILLED)

     
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=mask)
        
        # we want to find the side line and dash line
        segs = ld.line_segments(mask, 2)
     
        detected_lines = []
        
        for segment in segs:
            m, b = ld.ransac_method(segment)
    #         print("Equation line: y1 = %fx + %f" % (m, b))
            line = ld.end_start_points(m, b, im_h) # img.shape[1])
    
            #creating the custom message
            line_parameters = Line()
            line_parameters.slope = m
            line_parameters.intercept = b
            line_parameters.height = im_h
            line_parameters.width = im_w
            detected_lines.append(line_parameters)
                             
        
        ransac_lines = ld.draw_lines(img, detected_lines)
            
        try:
            # Ransac 
            img = self.bridge.cv2_to_imgmsg(ransac_lines, "rgb8")
            bw_img = self.bridge.cv2_to_imgmsg(mask, "mono8")
            
            self.lane_detection_pub.publish(img)
            self.bw_detection_pub.publish(bw_img)
            line_set = LineSet()
            line_set.line_set = detected_lines
            self.line_parameters_pub.publish(line_set)
             
        except CvBridgeError as e:
            print(e)
        
        
def main(args):
    rospy.init_node('lane_detector', anonymous=True)
    lane_detector = lane_detection()
    
    # Test case
#     lane_detector.callback(1)
    
    try:
        rospy.spin()       
    except KeyboardInterrupt:
        print("Shutting down")
#     cv2.destroyAllWindows()    
    
    # Test case
#     while True:
#         lane_detector.callback(1)
    
    
if __name__ == '__main__':
    main(sys.argv)

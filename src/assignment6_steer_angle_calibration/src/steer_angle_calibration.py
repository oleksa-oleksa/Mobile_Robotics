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


class lane_detection:
    def __init__(self):
        self.lane_detection_pub = rospy.Publisher("/image_processing/lane", Image, queue_size=10)
        
        self.bridge = CvBridge()
        self.lane_detection_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
 
    def callback(self, data):
        
        try:
            img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        
        
        
            
        try:
            # Test Unit
            #img = self.bridge.cv2_to_imgmsg(img, "rgb8")

            # Ransac 
            img = self.bridge.cv2_to_imgmsg(ransac_lines, "rgb8")
            self.lane_detection_pub.publish(img)
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

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image


def main():
    rospy.init_node("whoop_woop", anonymous=True)
    assert len(sys.argv) == 2
    image_path = sys.argv[1]
    print("Sending {}".format(image_path))
    im = cv2.imread(image_path)
    bridge = CvBridge()
    image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
    msg = bridge.cv2_to_imgmsg(im, "rgb8")
    while not rospy.is_shutdown():
        image_pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
  main()

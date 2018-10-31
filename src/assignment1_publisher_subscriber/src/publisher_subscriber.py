#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32

yaw_publisher = rospy.Publisher("assignment1_publisher_subscriber", String, queue_size=10)

def callback(raw_msg):
    msg_content = "{}: I heard {}".format(rospy.get_caller_id(), raw_msg.data)
    rospy.loginfo(msg_content)
    yaw_publisher.publish(msg_content)

    
def subscriber():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("yaw", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()

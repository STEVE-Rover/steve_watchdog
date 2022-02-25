#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

rospy.init_node("heartbeat")
rate = rospy.get_param("~rate", 2)
heartbeat_pub = rospy.Publisher("heartbeat", Empty, queue_size=1)
rospy.loginfo("Heartbeat ready")

if __name__ == "__main__":
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        heartbeat_pub.publish(Empty())
        r.sleep()
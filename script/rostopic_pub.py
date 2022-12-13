#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Vector3Stamped

if __name__ == "__main__":
    rospy.init_node('static_publisher')
    rate = rospy.Rate(50.0)

    pub = rospy.Publisher('/n3ctrl/idling', Vector3Stamped, queue_size=10)

    msg = Vector3Stamped()
    msg.header.frame_id = "idling"
    while not rospy.is_shutdown():

        pub.publish(msg)
        rate.sleep()


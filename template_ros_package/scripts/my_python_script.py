#! /usr/bin/env python

import rospy

rospy.init_node('THE_NAME_OF_THE_NODE')
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    print("Hello World")
    rate.sleep()
#! /usr/bin/env python  

import rospy

if __name__ == "__main__":
    rospy.init_node('THE_NAME_OF_THE_NODE')
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        print("Hello World")
        rate.sleep()
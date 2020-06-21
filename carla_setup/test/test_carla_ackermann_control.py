#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive

def talker():
    pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive,
                          queue_size=10) #Problem with this line
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.25) # 0.25hz = 4s
    msg = AckermannDrive()
    while not rospy.is_shutdown():
        msg.speed = 10.0 - msg.speed
        rospy.loginfo('Set speed to: %f', msg.speed)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

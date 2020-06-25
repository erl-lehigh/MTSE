#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive

def talker():
    #Creates a publisher node, broadcasting the AckermannDrive messages found in /carla/ego_vehicle/ackermann_cmd
    pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)
    #Initializes the node on which to broadcast
    rospy.init_node('talker', anonymous=True)
    #Sets how often it broadcasts
    rate = rospy.Rate(0.25) # 0.25hz = 4s
    #Stores the AcermanDrive data into a message object
    msg = AckermannDrive()
    while not rospy.is_shutdown():
        #Changes the speed
        msg.speed = 1.0 - msg.speed
        msg.steering_angle = 1.0 - msg.steering_angle
        #Broadcasts text
        rospy.loginfo('Set speed to: %f, and steering to %f', msg.speed, msg.steering_angle)
        #Broadcasts the message
        pub.publish(msg)
        #Sleeps for time equal to the rate
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

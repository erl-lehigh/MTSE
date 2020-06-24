#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive

def speed_talker():
    #Creates a publisher node, broadcasting the AckermannDrive messages found in /carla/ego_vehicle/ackermann_cmd
    pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)
    #Initializes the node on which to broadcast
    rospy.init_node('speed_talker', anonymous=True)
    #Sets how often it broadcasts
    rate = rospy.Rate(0.25) # 0.25hz = 4s
    #Stores the AcermanDrive data into a message object
    msg = AckermannDrive()
    while not rospy.is_shutdown():
        #Changes the speed
        msg.speed = 10.0 - msg.speed
        #Broadcasts text
        rospy.loginfo('Set speed to: %f', msg.speed)
        #Broadcasts the message
        pub.publish(msg)
        #Sleeps for time equal to the rate
        rate.sleep()

def steering_talker():
    pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)
    rospy.init_node('steering_talker', anonymous=True)
    rate = rospy.Rate(0.25) # 0.25hz = 4s
    msg = AckermannDrive()
    while not rospy.is_shutdown():
        msg.speed = 10.0 - msg.steering_angle
        rospy.loginfo('Set steering angle to: %f', msg.steering_angle)
        pub.publish(msg)
        rate.sleep()

def speed_listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('speed_listener', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, callback)
    speed = AckermannDrive().speed
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def steering_listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('steering_listener', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, callback)
    steering = AckermannDrive().steering_angle
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        speed_talker()
        steering_talker()
        speed_listener()
        steering_listener()
    except rospy.ROSInterruptException:
        pass

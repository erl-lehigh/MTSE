#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

from purepursuit import PurePursuit
from shapely.geometry import Point, LineString

# new line for test:
#from purepursuit import test_purepursuit



class PUREPURSUITROSNode(object):
    '''TODO:
    '''

    def __init__(self):
        '''TODO:
        '''
        #self.purepursuit = PurePursuit()

        # Initialize a path:
        self.path = None

        # Initialize the speed of the vehicle
        self.speed = 3

        self.steering_angle = 0


        self.node_name = rospy.get_name()

        # Read parameters
        #TODO: read parameters
        # second parameter is default value
        self.example = rospy.get_param('~parameter', 0)

        # Create publishers
        # TODO:
        # second parameter is the message type for the topic
        self.example_pub  = rospy.Publisher('speed_command', AckermannDrive,
                                            queue_size=1)

        # Create subscribers
        #function = print(str(instance_of_PurePursuit.compute_speed))
        self.timer = rospy.Timer(rospy.Duration(1),self.call_purepursuit , oneshot=False)

        self.example_sub = rospy.Subscriber('subscriber_example', Path,
                                            self.example_callback)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def example_callback(self, msg):
        '''Example message callback.'''
        print('Message received.')
        #
        pose_list = []

        for pose in msg.poses:
            #self.path = msg.poses[0].location.x, msg.poses[0].location.y
            pose_list.append((pose.pose.position.x, pose.pose.position.y))

        self.path = LineString(pose_list)

    def call_purepursuit(self, event=None):
        #path = LineString([(1, 1), (8, 4)]) # static path given for test
        #speed = 3   # given vehicle speed
        if self.path is None:
            #self.speed = 0
            msg = AckermannDrive()
            msg.speed = 0
            msg.steering_angle = 0 # since there is no path
            self.example_pub.publish(msg)
        else:
            msg = AckermannDrive()
            msg.speed = self.speed
            msg.steering_angle = self.steering_angle
            self.example_pub.publish(msg)

        vehicle_cords = Point(2,3)   # initial vehicle coordinates
        theta = 0  # rad
        instance_of_PurePursuit = PurePursuit(self.path, self.speed,vehicle_cords, theta)


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('purepursuit', anonymous=False)
    # Create the node object
    _ = PUREPURSUITROSNode()
    # Keep the node alive
    rospy.spin()

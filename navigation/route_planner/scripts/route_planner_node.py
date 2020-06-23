#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path

from route_planner import RoutePlanner


class RoutePlannerNode(object):
    '''TODO:
    '''

    def __init__(self):
        '''TODO:
        '''
        self.node_name = rospy.get_name()

        # Read parameters
        #TODO: read parameters
        # second parameter is default value
        self.example = rospy.get_param('~parameter', 0)

        # Create publishers
        # TODO:
        # second parameter is the message type for the topic
        self.example_pub  = rospy.Publisher('publisher_example', Path,
                                            queue_size=1)

        # Create subscribers
        self.example_sub = rospy.Subscriber('~subscriber_example', Path,
                                            self.example_callback)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def example_callback(self, msg):
        '''Example message callback.'''
        print('Message received.')


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=False)
    # Create the node object
    _ = RoutePlannerNode()
    # Keep the node alive
    rospy.spin()

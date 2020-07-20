#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped

from purepursuit import PurePursuit
from shapely.geometry import Point, LineString

# new line for test:
#from purepursuit import test_purepursuit

class TESTPUREPURSUITROSNode(object):
    '''
    A class to represent a TESTPUREPURSUITROS Node in ROS

    Attributes
    ----------
    node_name : str
        name of the node
    example : # ? not sure?
        #
    example_pub : rospy.Publisher
        a variable called "example_pub" that holds an instance of the class rospy.Publisher
    timer : rospy.Timer
        a variable called "timer" that holds an instance of the class rospy.Timer


    Methods
    -------
    call_pubpath(event=None):
        publishes path (coordinates) /poses to be tracked by PurePursuit algorithm

    '''

    def __init__(self):
        '''
        Constructs all the necessary attributes for the TESTPUREPURSUITROSNode object.

        Parameters
        ----------
        None

        '''

        self.node_name = rospy.get_name()

        # Read parameters
        #TODO: read parameters
        # second parameter is default value
        self.example = rospy.get_param('~parameter', 0)

        # Create publishers
        # second parameter is the message type for the topic
        self.example_pub  = rospy.Publisher('planned_path', Path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1),self.call_pubpath , oneshot=True)
        rospy.loginfo('[%s] Node started!', self.node_name)


    def call_pubpath(self, event=None):
        '''
        Publishes path (coordinates) /poses to be tracked by PurePursuit algorithm

        Parameters
        ----------
        event=None : # not sure ??
            #

        Returns
        -------
        None
        '''
        path = Path()
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = 1.0
        goal.pose.position.y = 2.0
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0


        path.poses.append(goal)

	goal = PoseStamped()

	goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = 3.0
        goal.pose.position.y = 5.0
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

	path.poses.append(goal)


        self.example_pub.publish(path)


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('test_purepursuit', anonymous=False)
    # Create the node object
    _ = TESTPUREPURSUITROSNode()
    # Keep the node alive
    rospy.spin()

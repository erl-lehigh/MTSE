#! /usr/bin/env python

'''
Test Pure Pusrsuit Node
'''

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from purepursuit import PurePursuit
from test_paths import test_paths


class TestPurePursuitNode(object):
    '''
    A class to represent a TestPurePursuitNode

    Attributes
    ----------
    node_name : str
        name of the node
    path_pub : rospy.publisher
        a variable called "command_pub" that holds an instance of the class
        rospy.Publisher
    timer : rospy.Timer
        a variable called "timer" that holds an instance of the class
        rospy.Timer

    Methods
    -------
    publish_path(event=None)
        Publishes the path coordinates for the vehcile to track/follow
    '''

    def __init__(self):
        '''
        Constructs all the necessary attributes for the TestPurePursuitNode
        object.

        Parameters
        ----------

        '''
        self.node_name = rospy.get_name()

        # Create publishers
        self.path_pub  = rospy.Publisher('planned_path', Path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_path,
                                 oneshot=False)
        rospy.loginfo('[%s] Node started!', self.node_name)

    def publish_path(self, event=None):
        '''
        Publishes the path coordinates for the vehcile to track/follow.

        Parameters
        ----------
        event=None : rospy.TimerEvent
            information about the event that generated this call

        Returns
        -------
        None
        '''
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "world"


        path_coords = test_paths[5]
        # The list above is referencing from a list of tests that are in
        # a python folder in the test directory
        '''
            paths
                5 - +x
                6 - -x
                7 - +y
                8 - -y
        '''
        for x, y in path_coords:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"

            pose.pose.position.x = x
            pose.pose.position.y = y

            path.poses.append(pose)

        self.path_pub.publish(path)
        rospy.loginfo('Published path!')


if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('test_purepursuit', anonymous=False)
    # create the node object
    _ = TestPurePursuitNode()
    # keep the node alive
    rospy.spin()

#! /usr/bin/env python

'''
Test Pure Pusrsuit Node
'''

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from purepursuit import PurePursuit

class CarlaWaypointNode(object):
    '''
    Uses information from the Carla simulator to create a path.

    Attributes
    ----------
    

    Methods
    -------
    
    Notes
    -----
    - if the car is too fast or too slow there might be an issue
    - change the timer time if that is the case

    '''

    def __init__(self):
        '''
        Constructs all the necessary attributes for the TestPurePursuitNode
        object.

        Parameters
        ----------

        '''
        self.node_name = rospy.get_name()
        self.message_path = Path()

        # Create subs
        self.waypoint_sub = rospy.Subscriber('/carla/ego_vehicle/waypoints',
                                             Path, self.set_path)
        # If the above gives issues, remove first /
        # Check rqt for figuring if it is working

        # Create publishers
        self.path_pub  = rospy.Publisher('planned_path', Path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_path,
                                 oneshot=False)
        rospy.loginfo('[%s] Node started!', self.node_name)

    def set_path(self, msg):
        '''
        Just set the msg to self.message_path, so it can be republished in published path
        '''
        self.message_path = msg

    def publish_path(self, event=None):
        '''
        Publishes the path coordinates for the vehcile to track/follow.

        Parameters
        ----------
        event=None : ?? not sure
            # not sure ???

        Returns
        -------
        None
        '''
        if self.message_path != Path():
            self.path_pub.publish(self.message_path)


if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('carla_waypoint', anonymous=False)
    # create the node object
    _ = CarlaWaypointNode()
    # ceep the node alive
    rospy.spin()
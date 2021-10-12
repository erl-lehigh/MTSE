#! /usr/bin/env python

'''
Static Waypoint Node - Carla (Initial Map)
'''
import pandas as pd

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

        # importing the static path
        path_locations_df = pd.read_csv('~/mtse_catkin/src/MTSE/navigation/purepursuit/test/SDC_carla_static_path.csv',
                                          header=None)
        # convert the dataframe to pairs of x and y locations
        # will be converted to path in public method
        self.path_locations = path_locations_df.values.tolist()



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
        event=None : ?? not sure
            # not sure ???

        Returns
        -------
        None
        '''
        path = Path()                       # initial path value
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for point in self.path_locations:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = -point[1]
            path.poses.append(pose)
        self.path_pub.publish(path)
            


if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('carla_waypoint', anonymous=False)
    # create the node object
    _ = CarlaWaypointNode()
    # ceep the node alive
    rospy.spin()
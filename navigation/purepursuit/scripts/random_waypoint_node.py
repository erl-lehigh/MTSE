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

        # Initial Carla Code
        client = carla.Client("localhost", 2000)    #connect to server
        client.set_timeout(10)                      #set timeout
        world = client.load_world('Town03')         #get access to the world info
        self.world_map = world.get_map()                 #needed for waypoints (map)
        starting_point = carla.Transform(carla.Location(x=220.053, y=-5, z=0),
                                         carla.Rotation(pitch=0, yaw=180, roll=0))
        self.waypoint = self.world_map.get_waypoint(starting_point.location)
                                                    #set initial location


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
        path_forward = self.waypoint.next(10)
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map" # If this give issues change to "world"
        for point in path_forward:  #uses waypoint locations to make a path
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"   # If this give issues change to "world"
            pose.pose.position.x = point.location.x
            pose.pose.position.y =  point.location.y
            path.poses.append(pose)
        self.waypoint = path_forward[-1]
        self.path_pub.publish(path)


if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('carla_waypoint', anonymous=False)
    # create the node object
    _ = CarlaWaypointNode()
    # ceep the node alive
    rospy.spin()
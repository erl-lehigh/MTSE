#! /usr/bin/env python

'''
Test Pure Pusrsuit Node
'''

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

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
        rospy.loginfo('code for rwp started!')
        self.node_name = rospy.get_name()

        # Initial Carla Code
        client = carla.Client("localhost", 2000)    #connect to server
        client.set_timeout(2)                      #set timeout
        #world = client.load_world('Town03')         #get access to the world info
        world = client.get_world()
        '''
        load_world loads a completely new world, the current world is killed.
        get_world loads the current world
        '''
        self.world_map = world.get_map()                 #needed for waypoints (map)
        # len_wps = len(self.world_map.generate_waypoints(1))
        # rospy.loginfo('[\n\nfrom random\n\n\Number of waypoints] %d', len_wps)
        self.all_waypoints = self.world_map.generate_waypoints(1)
        self.vehicle_location = carla.Location(x=210.053, y=5, z=0)
        self.vehicle_euler_angles = (0,180,0)
        self.waypoint = self.find_closest(self.vehicle_location)
                                                    #set initial location

        # Create subscribers
        self.vehicle_location_sub = rospy.Subscriber('vehicle_location_pub',
                                                     PoseStamped,
                                                     self.set_vehicle_information)

        # Create publishers
        self.path_pub  = rospy.Publisher('planned_path', Path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_path,
                                 oneshot=False)
        rospy.loginfo('[%s] Node started!', self.node_name)

    def set_vehicle_information(self, msg):
        '''
        get vehicle location as posestamped and then save the location as a 
        carla location obj
        '''
        x = msg.pose.position.x 
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.vehicle_location = carla.Location(x=x, y=y, z=z)
        orientation = msg.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.vehicle_euler_angles = euler_from_quaternion(orientation_list)
        next_waypoint = self.find_closest(self.vehicle_location)

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
        #path_forward = self.waypoint.next_until_lane_end(100)
        # wps = self.world_map.generate_waypoints(1)
        
        next_distance = 2
        path_forward = self.waypoint.next_until_lane_end(next_distance)
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map" # If this give issues change to "world"
        for point in path_forward:  #uses waypoint locations to make a path
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"   # If this give issues change to "world"
            pose.pose.position.x = point.transform.location.x
            pose.pose.position.y = point.transform.location.y
            path.poses.append(pose)
            rospy.loginfo("%s", point.lane_type)
        next_waypoint = self.find_closest(self.vehicle_location)
        # next_waypoint = self.find_closest(self.vehicle_location, waypoints=path_forward)
        # if next_waypoint == []:
        #     next_waypoint = self.find_closest(self.vehicle_location)
        # if next_waypoint != []:
        #     self.waypoint = next_waypoint
        self.path_pub.publish(path)

    def find_closest(self, location, distance=2, waypoints=[]):
        '''
        Goes through the waypoints and finds the waypoint that is less that
        {distance (m)} away.
        '''
        if waypoints == []:
            waypoints = self.all_waypoints
        for waypoint in waypoints:
            # if waypoint.transform.location.x > 210:
            #     continue
            if waypoint.lane_type in ["Restricted", "Border", "Sidewalk", "Parking", "Median"]:
                continue
            dist = location.distance(waypoint.transform.location)
            if dist <= distance:
                return waypoint
        # return self.find_closest(location, distance=distance+1, waypoints=waypoints)
        return []

if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('carla_waypoint', anonymous=False)
    # create the node object
    _ = CarlaWaypointNode()
    # ceep the node alive
    rospy.spin()
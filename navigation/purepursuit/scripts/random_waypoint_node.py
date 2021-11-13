#! /usr/bin/env python

'''
Test Pure Pusrsuit Node
'''
import math
import pandas as pd
import networkx as nx
import pickle
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
        waypoints_dict = {}
        junctions_dict = {}
        self.all_waypoints = self.world_map.generate_waypoints(1)
        all_points_x = []
        all_points_y = []
        all_points_is_junction = []
        junctions = []
        start_x = []
        start_y = []
        end_x = []
        end_y = []
        junction_id = []
        for wp in self.all_waypoints:
            all_points_x.append(wp.transform.location.x)
            all_points_y.append(wp.transform.location.y)
            waypoints_dict[wp.id] = wp
            in_junction = wp.is_junction
            all_points_is_junction.append(in_junction)
            if in_junction:
                junctions.append(wp.get_junction())
        num_junc_waypoints = 0
        num_junc_overlap = 0
        junc_wp_dict = {}
        junc_ids = []
        for junct in junctions:
            if junct.id not in junction_id:
                wp_pairs = junct.get_waypoints(carla.LaneType.Any)
                for wp_pair in wp_pairs:
                    start_id = wp_pair[0].id
                    end_id = wp_pair[1].id
                    junc_ids.append(end_id)
                    junc_ids.append(start_id)
                    num_junc_waypoints += 2
                    if start_id in waypoints_dict.keys():
                        num_junc_overlap += 1
                    if end_id in waypoints_dict.keys():
                        num_junc_overlap += 1
                    if start_id in junctions_dict.keys():
                        junctions_dict[start_id].append(end_id)
                    else:
                        junctions_dict[start_id] = [end_id] 
                    junc_wp_dict[start_id] = wp_pair[0]
                    junc_wp_dict[end_id] = wp_pair[1]
                    start_x.append(wp_pair[0].transform.location.x)
                    start_y.append(wp_pair[0].transform.location.y)
                    end_x.append(wp_pair[1].transform.location.x)
                    end_y.append(wp_pair[1].transform.location.y)
                    junction_id.append(junct.id)

        location_graph = nx.Graph()
        waypoint_graph = nx.Graph()

        wp_ids = list(waypoints_dict.keys())
        wp_ids.extend(junc_ids)
        ## Dictionary version
        # for wp_id in wp_ids:
        #     if wp_id in waypoints_dict.keys():
        #         waypoint_graph.add_nodes_from([(wp_id, {'waypoint': waypoints_dict[wp_id]})])
        #         location_graph.add_nodes_from([(wp_id, {
        #             'x': waypoints_dict[wp_id].transform.location.x,
        #             'y': waypoints_dict[wp_id].transform.location.y
        #         })])
        #     elif wp_id in junc_wp_dict.keys(): 
        #         waypoint_graph.add_nodes_from([(wp_id, {'waypoint': junc_wp_dict[wp_id]})])
        #         location_graph.add_nodes_from([(wp_id, {
        #             'x': junc_wp_dict[wp_id].transform.location.x,
        #             'y': junc_wp_dict[wp_id].transform.location.y
        #         })])
        ## Tuple Version of x, y
        for wp_id in wp_ids:
            if wp_id in waypoints_dict.keys():
                waypoint_graph.add_nodes_from([(wp_id, {'waypoint': waypoints_dict[wp_id]})])
                location_graph.add_nodes_from([(wp_id, {'location': (waypoints_dict[wp_id].transform.location.x, waypoints_dict[wp_id].transform.location.y)})])
            elif wp_id in junc_wp_dict.keys(): 
                waypoint_graph.add_nodes_from([(wp_id, {'waypoint': junc_wp_dict[wp_id]})])
                location_graph.add_nodes_from([(wp_id, {'location': (junc_wp_dict[wp_id].transform.location.x, junc_wp_dict[wp_id].transform.location.y)})])
        for wp_id in wp_ids:
            if wp_id in waypoints_dict.keys():
                wp = waypoints_dict[wp_id]
                next_wps = wp.next(.2)
                for next_wp in next_wps:
                    if next_wp.id not in wp_ids:
                        waypoint_graph.add_nodes_from([(next_wp.id, {'waypoint': next_wp})])
                        location_graph.add_nodes_from([(next_wp.id, {'location': (next_wp.transform.location.x, next_wp.transform.location.y)})])
                    location_graph.add_edge(wp.id, next_wp.id)
                    waypoint_graph.add_edge(wp.id, next_wp.id)
                if wp_id in junctions_dict.keys():
                    if junctions_dict[wp.id][0] not in wp_ids:
                        waypoint_graph.add_nodes_from([(junctions_dict[wp.id][0], {'waypoint': junc_wp_dict[junctions_dict[wp.id][0]]})])
                        location_graph.add_nodes_from([(junctions_dict[wp.id][0], {'location': (junc_wp_dict[junctions_dict[wp.id][0]].transform.location.x, junc_wp_dict[junctions_dict[wp.id][0]].transform.location.y)})])
                    location_graph.add_edge(wp.id, junctions_dict[wp.id][0])
                    waypoint_graph.add_edge(wp.id, junctions_dict[wp.id][0])
            elif wp_id in junc_wp_dict.keys():
                wp = junc_wp_dict[wp_id]
                next_wps = wp.next(.2)
                for next_wp in next_wps:
                    if next_wp.id not in wp_ids:
                        waypoint_graph.add_nodes_from([(next_wp.id, {'waypoint': next_wp})])
                        location_graph.add_nodes_from([(next_wp.id, {'location': (next_wp.transform.location.x, next_wp.transform.location.y)})])
                    location_graph.add_edge(wp.id, next_wp.id)
                    waypoint_graph.add_edge(wp.id, next_wp.id)
                if wp_id in junctions_dict.keys():
                    if junctions_dict[wp.id][0] not in wp_ids:
                        waypoint_graph.add_nodes_from([(junctions_dict[wp.id][0], {'waypoint': junc_wp_dict[junctions_dict[wp.id][0]]})])
                        location_graph.add_nodes_from([(junctions_dict[wp.id][0], {'location': (junc_wp_dict[junctions_dict[wp.id][0]].transform.location.x, junc_wp_dict[junctions_dict[wp.id][0]].transform.location.y)})])
                    location_graph.add_edge(wp.id, junctions_dict[wp.id][0])
                    waypoint_graph.add_edge(wp.id, junctions_dict[wp.id][0])


        pickle.dump(location_graph, open('location_graph.txt', 'w+'))
        # pickle.dump(waypoint_graph, open('waypoint_graph.txt', 'w+'))

        # with open('wp_dict.txt', 'w+') as fp:
        #     fp.write(str(waypoints_dict))
        # with open('junctions_dict.txt', 'w+') as fp:
        #     fp.write(str(junctions_dict))
        all_dict = {
            'x': all_points_x,
            'y': all_points_y,
            'in_junction': in_junction
        }
        junction_dict = {
            'junction_id': junction_id,
            'start x': start_x,
            'start y': start_y,
            'end x': end_x,
            'end y': end_y
        }
        pd.DataFrame.to_csv(pd.DataFrame(data=all_dict), 'all_points.csv', index=False)
        pd.DataFrame.to_csv(pd.DataFrame(data=junction_dict), 'junction_points.csv', index=False)
        

        self.vehicle_location = carla.Location(x=210.053, y=5, z=0)
        self.vehicle_euler_angles = (0,180,0)
        self.view_range = 1.55 #the range in either direction which a car
        # is concerned with.
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
        print('Angle: ' + str(self.vehicle_euler_angles))
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
            # rospy.loginfo("%s", point.lane_type)
        print(self.vehicle_location.x)
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
            if dist <= distance and self.is_waypoint_in_view(waypoint.transform.location):
                return waypoint
        # return self.find_closest(location, distance=distance+1, waypoints=waypoints)
        return []

    def is_waypoint_in_view(self, location):
        '''
        Uses the waypoint locations and calculates the angle to the current location
        (where the +X axis is 0 radians). Once the angle is calculated, if it is in
        the range of the vehicle's view, then true is returned (else false)
        '''
        delta_x = location.x - self.vehicle_location.x
        delta_y = location.y - self.vehicle_location.y
        angle = math.atan2(delta_y, delta_x)
        difference = angle - self.vehicle_euler_angles[2]
        if abs(difference) >= math.pi:
            if difference < 0:
                angle += 2*math.pi
            elif difference > 0:
                angle -= 2*math.pi
        output = "[%0.3f, %0.3f] Calculated: %0.3f - TF: %0.3f" % (delta_x, delta_y,angle, self.vehicle_euler_angles[2])
        print(output)
        # Note: seems that the car is -pi but that math doesn't work with values in Q2
        # the Euler angle should align with the above because both are atan2 based
        if abs(angle - self.vehicle_euler_angles[2]) > self.view_range:
            return False
        return True




if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('carla_waypoint', anonymous=False)
    # create the node object
    _ = CarlaWaypointNode()
    # ceep the node alive
    rospy.spin()
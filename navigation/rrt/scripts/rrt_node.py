#! /usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tr

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid

import numpy as np

from rrt import RRTPlanner, DynamicDubinsVehicle, DubinsState


class RRTNode(object):
    '''TODO: docstring
    '''

    def __init__(self):
        '''TODO: docstring
        '''
        self.node_name = rospy.get_name()

        # Read parameters
        self.rate = rospy.get_param('~rate', 10)
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')
        max_iterations = rospy.get_param('~max_iterations', 2000)
        gamma = rospy.get_param('~gamma', 40)
        dist_threshold = rospy.get_param('~dist_threshold', 5)
        angle_threshold = rospy.get_param('~angle_threshold', 0.34907)
        step_size = rospy.get_param('~step_size', 10.0)
        self.goal_bias = rospy.get_param('~goal_bias', 0.3)

        self.period = rospy.Duration(1.0 / self.rate)

        # Create path planner
        self.rrt_planner = RRTPlanner(DynamicDubinsVehicle(None),
                                      max_iterations, gamma)
        self.rrt_planner.dist_threshold = dist_threshold
        self.rrt_planner.angle_threshold = angle_threshold
        self.rrt_planner.step_size = step_size
        # Set collision checker (function) for the RRT* path planner
        self.rrt_planner.check_path = self.check_path
        # Set sampler (function) for the RRT* path planner
        self.rrt_planner.sample = self.sample

        self.goal = None
        self.costmap = None

        self.header = Header(frame_id=self.parent_frame)

        self.best_path = Path()
        self.best_path.header.frame_id = self.parent_frame

        # Create publishers
        self.path_pub  = rospy.Publisher('planned_path', Path,
                                         queue_size=1)
        # Create subscribers
        self.costmap_sub = rospy.Subscriber('costmap', OccupancyGrid,
                                            self.set_costmap)
        self.goal_sub = rospy.Subscriber('goal_point', PoseStamped, self.set_goal)
        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Create timers
        self.timer = rospy.Timer(self.period, self.control_loop)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def set_goal(self, msg):
        '''TODO: docstring
        Converte the user input goal from orientation into quaternion
        Parameters
        ----------
        msg:
        coordinate for the goal

        Return
        ------
        None
        '''
        assert msg.header.frame_id == self.parent_frame
        quaternion = msg.pose.orientation
        quaternion = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        _, _, yaw = tr.euler_from_quaternion(quaternion)
        self.goal = DubinsState(x=msg.pose.position.x, y=msg.pose.position.y,
                                yaw=yaw, v=0, omega=0)

    def set_costmap(self, msg):
        '''TODO: docstring
        '''
        assert msg.header.frame_id == self.parent_frame
        self.costmap = msg

    def get_vehicle_pose(self):
        '''TODO: docstring
        '''
        try:
            trans = self.tf_buffer.lookup_transform(self.child_frame,
                                                    self.parent_frame,
                                                    rospy.Time.now(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        quaternion = trans.transform.rotation
        quaternion = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        _, _, orientation = tr.euler_from_quaternion(quaternion)
        return (trans.transform.translation.x, trans.transform.translation.y,
                orientation)

    def check_path(self, path):
        '''
        Iterates through all the coordinates in the path check if that
        position is ocupied or not

        Parameters
        ----------
        path: list of coordinates
            the coordinates for the path

        Returns
        -------
        true or false
            true if the path is collition free, false if not
        '''
        ogm = self.costmap
        if ogm is None:
            rospy.logwarn('No costmap set!')
            return

        height = ogm.info.height
        width = ogm.info.width
        grid = np.asarray(ogm.data).reshape((height, width))
        positionX = ogm.info.origin.position.x
        positionY = ogm.info.origin.position.y
        resolution = ogm.info.resolution
        for dubinsState in path:
            px = dubinsState.x
            py = dubinsState.y
            x = (px - positionX)/resolution
            y = (py - positionY)/resolution
            if(0 <= x <= width or 0 <= y <= height):
                if(grid[int(x),int(y)] >= 100):
                    return False
                elif(0 <= grid[int(x),int(y)] < 100):
                    continue
                else:
                    return False
            else:
                return False
        return True

    def sample(self, goal):
        '''TODO: docstring
        '''
        if self.costmap is None:
            rospy.logwarn('No costmap set!')
            return

        if np.random.uniform() < self.goal_bias:
            return goal

        origin = self.costmap.info.origin.position
        width = self.costmap.info.width * self.costmap.info.resolution
        height = self.costmap.info.height * self.costmap.info.resolution

        x_min, x_max = origin.x - width / 2., origin.x + width / 2.
        x = np.random.uniform(x_min, x_max)

        y_min, y_max = origin.y - height / 2., origin.y + height / 2.
        y = np.random.uniform(y_min, y_max)

        return DubinsState(x=x, y=y, yaw=0, v=0, omega=0)

    def control_loop(self, event):
        '''TODO: docstring
        '''
        if self.goal is None:
            rospy.logwarn('No goal set!')
            return

        if self.costmap is None:
            rospy.logwarn('No costmap set!')
            return

        # Set current state (pose only) of the vehicle
        x, y, yaw = self.get_vehicle_pose()
        self.rrt_planner.vehicle.current_state = DubinsState(x, y, yaw, 0, 0)
        # Plan path from current state to goal
        path = self.rrt_planner.plan(self.goal)

        # Publish returned path
        self.best_path.header.stamp = rospy.Time.now()
        self.best_path.poses = []
        self.header.stamp = rospy.Time.now()
        for x, y, yaw, _, _ in path:
            pose = PoseStamped(header=self.header)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = tr.quaternion_from_euler(0, 0, yaw)
            self.best_path.poses.append(pose)
        self.path_pub.publish(self.best_path)


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('rrt', anonymous=False)
    # Create the node object
    _ = RRTNode()
    # Keep the node alive
    rospy.spin()

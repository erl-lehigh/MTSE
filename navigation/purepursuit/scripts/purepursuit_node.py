#! /usr/bin/env python
import math

import rospy
import tf2_ros
import tf.transformations as tr
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from shapely.geometry import LineString

from purepursuit import PurePursuit


class PurePursuitNode(object):
    '''
    A class to represent a pure pursuit node in ROS

    Attributes
    ----------
    node_name : str
        name of the node
    parent_frame : str
        parent Reference frame
    child_frame : str
        child Reference frame
    period : rospy.Duration
        specifies the duration, which is a period of time
    rate : float
        specifies the rate of the control loop and speed command publishing
    purepursuit : purepursuit.PurePursuit
        path tracker object using the pure pursuit method
    command_pub : rospy.Publisher
        the speed command publisher
    target_pub : rospy.Publisher
        the pure pursuit (moving) target publisher
    path_sub : rospy.Subscriber
        the subscriber for the tracked path
    lookahead_pud : nav_msgs.msg.Path
        Circle of radius of the lookahead centeredd at vehicle
    speed_visual_pub : nav_msgs.msg.Path
        Cross with points of length of the meters in the speed (m/s) centered at vehicle
    draw_speed_and_lookahead : Boolean
        Whether the shapes should be drawn or not.
    tf_buffer : tf2_ros.Buffer
        the transform listener buffer
    tf_listener : tf2_ros.TransformListener
        the listener for ROS transforms
    timer : rospy.Timer
        the control loop timer

    Methods
    -------
    set_path(msg):
        generates a path LineString (to be tracked) from a set of position
        coordinates (poses)
    control_loop(event=None):
        publishes AckermannDrive msg consisting of the computed vehicle speed
        and steering angle
    get_vehicle_pose():
        returns the vehicle coordinates (position) for pure pursuit computation
    '''

    def __init__(self):
        '''
        Initializes the purepursuit node that subscribes to a planned path
        topic, computes steering angle commands based on the received paths, and
        and publishes Ackermann commands. The goal point computed by the
        purepursuit method is also publised.
        '''
        self.node_name = rospy.get_name()

        # Read parameters
        self.rate = rospy.get_param('~rate', 1)
        self.parent_frame = rospy.get_param('~parent_frame', 'map')
        self.child_frame = rospy.get_param('~child_frame', 'ego_vehicle_filtered')
        wheelbase = rospy.get_param('~wheelbase', 1)
        lookahead_min = rospy.get_param('~lookahead_min', 3)
        lookahead_max = rospy.get_param('~lookahead_max', 12)
        lower_threshold_v = rospy.get_param('~lower_velocity_threshold', 1.34)
        upper_threshold_v = rospy.get_param('~upper_velocity_threshold', 5.36)
        lookahead_gain = rospy.get_param('~lookahead_gain', 2.24)
        self.draw_speed_and_lookahead = rospy.get_param('~draw_speed_and_lookahead',
                                                   False)

        self.period = rospy.Duration(1.0 / self.rate)

        self.purepursuit = PurePursuit(wheelbase, lookahead_min,
                                       lookahead_max, lower_threshold_v,
                                       upper_threshold_v, lookahead_gain,
                                       speed=0)

        # Create publishers
        self.command_pub = rospy.Publisher('speed_command',
                                           AckermannDrive,
                                           queue_size=1)

        self.target_pub = rospy.Publisher('~/target', PoseStamped, queue_size=1)

        self.lookahead_pub = rospy.Publisher('lookahead_shape', Path,
                                            queue_size=1)
        # A publisher for the shape of the lookahead
        # ( circle with radius of the lookahead)
        self.speed_visual_pub = rospy.Publisher('speed_visual_pub',
                                                Path,
                                                queue_size=1)

        self.vehicle_location_pub = rospy.Publisher('vehicle_location_pub',
                                                    PoseStamped,
                                                    queue_size=1)

            # Create subscribers
        self.path_sub = rospy.Subscriber('planned_path', Path, self.set_path)
        self.speed_sub = rospy.Subscriber('reference_speed', Float64,
                                          self.set_speed)
        #this will hold Vcmd in .data public attribute

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.control_loop)

        rospy.logdebug('[%s] Node started!', self.node_name)

    def get_vehicle_pose(self):
        '''
        Returns the vehicle's pose (position and orientation).

        Parameters
        ----------
        None

        Returns
        -------
        tuple
            vehicle coordinates (x,y), orientation (angle) and quaterion
            representation for orientation
        Note
        ----
            The code in the try section had problems in its state in main.
            It is now correct here and in the fix/purepursuit_integration. This
            correct version of code is also posted in the TF guide on the drive
            and has a littl explanation as to why there might have been an issue.
        '''

        # order of tf lokup is parent, child time period
        try:
            trans = self.tf_buffer.lookup_transform(self.parent_frame,
                                                    self.child_frame,
                                                    rospy.Time(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as err:
            rospy.logdebug("e: %s", err)
            return
        quaternion_message = trans.transform.rotation
        quaternion = (quaternion_message.x, quaternion_message.y,
                      quaternion_message.z, quaternion_message.w)
        _, _, orientation = tr.euler_from_quaternion(quaternion)
        return (trans.transform.translation.x, trans.transform.translation.y,
                orientation, quaternion_message)

    def set_path(self, msg):
        '''
        Generates a path LineString (to be tracked) from a set of position
        coordinates (pose).

        Parameters
        ----------
        msg :  nav_msgs.msg.Path
            ROS navigation path message

        Returns
        -------
        None
        '''
        vehicle_pose = self.get_vehicle_pose()
        self.purepursuit.set_vehicle_pose(vehicle_pose)
        pose_list = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in msg.poses]
        self.purepursuit.path = LineString(pose_list)


    def set_speed(self, msg):    #Float64 is the msg that is passed
        '''
        This method changes the speed based on the speed commanded.
            It also changes the lookahead distance too.
        This method also changes the speed in accordance
            of the curvature (inverse proportional)

        Parameters
        ----------
        msgs : Float64
            the speed command

        Return
        ------
        none
        '''
        curvature = self.purepursuit.compute_curvature() #calc curvature
        new_speed = msg.data * (1 - curvature) # speed(inverse curvature)
        self.purepursuit.speed = new_speed
        self.purepursuit.update_lookahead(
                new_speed, self.purepursuit.lookahead_min,
                self.purepursuit.lookahead_max, self.purepursuit.lower_threshold_v,
                self.purepursuit.upper_threshold_v, self.purepursuit.lookahead_gain)
        rospy.logdebug('speed changed to %5.2f m/s', msg.data)
            #message on speed change

    def control_loop(self, event=None):
        '''
        The control loop computes the vehicle's speed and steering angle if path
        to track is set, and publishes AckermannDrive messages.

        Parameters
        ----------
        event=None : rospy.TimerEvent
            information about the event that generated this call

        Returns
        -------
        None
        '''
        msg = AckermannDrive(1.0, 0, 0, 0, 0)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.parent_frame 
        #The frame is the parent id so the below locations 
        #align with  get_vehicle_pose()
        vehicle_location_msg = PoseStamped()
        vehicle_location_msg.header.stamp = rospy.Time.now()
        vehicle_location_msg.header.frame_id = self.parent_frame
        if self.purepursuit.path is not None:
            locationOfVehicle = self.get_vehicle_pose()
            self.purepursuit.set_vehicle_pose(locationOfVehicle)
            position = self.purepursuit.future_point()
            pose_msg.pose.position.x = position.x
            pose_msg.pose.position.y = position.y
            msg.speed = self.purepursuit.speed
            vehicle_location_msg.pose.position.x = locationOfVehicle[0]
            vehicle_location_msg.pose.position.y = locationOfVehicle[1]
            vehicle_location_msg.pose.orientation = locationOfVehicle[3]
            rospy.logdebug(str(locationOfVehicle))
            rospy.logdebug('Vehicle - X:%5.2f Y:%5.2f',
                            locationOfVehicle[0],
                            locationOfVehicle[1])
            msg.steering_angle = self.purepursuit.compute_steering_angle()
            rospy.logdebug("AckDrvMsg: %5.2f m/s %5.2f rad",
                            msg.speed,
                            msg.steering_angle)
            self.command_pub.publish(msg)
            self.target_pub.publish(pose_msg)
            self.vehicle_location_pub.publish(vehicle_location_msg)

            #the published shapes are conditional
            if self.draw_speed_and_lookahead:
                self.publish_lookahead_shape()
                self.publish_speed_shape()

    def publish_lookahead_shape(self):
        '''
        Uses the vehicle location to draw a path circle around the
        vehicle. Publishes the shape.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
        la_distance = self.purepursuit.lookahead #lookahead distance
        # location of vehicle (center of circle)
        vehicle_location = self.get_vehicle_pose()
        # circle of lookahead radius
        lookahead_circle = Path()
        lookahead_circle.header.stamp = rospy.Time.now()
        lookahead_circle.header.frame_id = self.parent_frame
        theta = 0
        delta_theta = math.pi / 16
        end_theta = 2 * math.pi + delta_theta

        while theta <= end_theta and vehicle_location != None:
            pose = PoseStamped()
            pose.header = lookahead_circle.header
            pose.pose.position.x = (vehicle_location[0]
                                    + la_distance * math.cos(theta))
            pose.pose.position.y = (vehicle_location[1]
                                    + la_distance * math.sin(theta))
            lookahead_circle.poses.append(pose) # add pose to visualization path
            theta = theta + delta_theta # increment the theta
        self.lookahead_pub.publish(lookahead_circle)

    def publish_speed_shape(self):
        '''
        Uses the vehicle location to draw a cross with side lengths equal to
        the meters for the meters for second speed.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
        # location of vehicle (center of circle)
        vehicle_location = self.get_vehicle_pose()
        # create path
        speed_vis = Path()
        #circle of lookahead radius
        speed_vis.header.stamp = rospy.Time.now()
        speed_vis.header.frame_id = self.parent_frame
        speed = self.purepursuit.speed
        deltaList = [[1,0],[0,0],[-1,0],[0,0],[0,1],[0,0],[0,-1]]
            #list of locations for the X shape
        if vehicle_location != None:
            for delta in deltaList:
                pose = PoseStamped()
                #Position part
                pose.header.stamp = rospy.Time.now()
                #stamp
                pose.header.frame_id = self.parent_frame
                #reference
                pose.pose.position.x = vehicle_location[0] + speed * delta[0]
                #x location
                pose.pose.position.y = vehicle_location[1] + speed * delta[1]
                #y location
                speed_vis.poses.append(pose)
                #add location to path
        #Publish Shape
        self.speed_visual_pub.publish(speed_vis)

if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('purepursuit', anonymous=False)
    # add ', log_level=rospy.DEBUG' into the above if
    # if you would like the log information
    # create the node object
    _ = PurePursuitNode()
    # keep the node alive
    rospy.spin()
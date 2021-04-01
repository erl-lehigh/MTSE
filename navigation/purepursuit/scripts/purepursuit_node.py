#! /usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tr
import math

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
	Circle of radius of the lookahead
	Centered at vehicle
    speed_visual_pub : nav_msgs.msg.Path
	Cross with points of length of the meters in the speed (m/s)
	Centered at vehicle
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
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')
        lookahead = rospy.get_param('~lookahead', 4)
        wheelbase = rospy.get_param('~wheelbase', 1)
	lookahead_min = rospy.get_param('~lookahead_min', 3)
	lookahead_max = rospy.get_param('~lookahead_max', 12)
	lower_threshold_v = rospy.get_param('~lower_threshold_v', 1.34)
	upper_threshold_v = rospy.get_param('~upper_threshold_v', 5.36)
	lookahead_gain = rospy.get_param('~lookahead_gain', 2.24)

        self.period = rospy.Duration(1.0 / self.rate)

        self.purepursuit = PurePursuit(wheelbase, lookahead, lookahead_min, lookahead_max, lower_threshold_v, upper_threshold_v, lookahead_gain, speed=0)

        # Create publishers
        self.command_pub = rospy.Publisher('speed_command', 
					    AckermannDrive,
                                           queue_size=1)

        self.target_pub = rospy.Publisher('~/target', PoseStamped, queue_size=1)

	self.lookahead_pub = rospy.Publisher('lookahead_shape', 
						Path,
						queue_size=1)
	#^a publisher for the shape of the lookahead (basically circle with radius of the
	# lookahead
	self.speed_visual_pub = rospy.Publisher('speed_visual_pub', Path, queue_size=1)

	self.vehicle_location_pub = rospy.Publisher('vehicle_location_pub', PoseStamped, queue_size=1)

        # Create subscribers


	self.path_sub = rospy.Subscriber('planned_path', 
					Path, 
					self.set_path)

	self.speed_sub = rospy.Subscriber('reference_speed', 
						Float64, 
						self.set_speed) 
				#this will hold Vcmd in .data public attribute

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.control_loop)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def get_vehicle_pose(self):
        '''
        Returns the vehicle's pose (position and orientation).

        Parameters
        ----------
        None

        Returns
        -------
        tuple
            vehicle coordinates (x,y) and orientation (angle) and quaterion (direction for rvix)
        '''


        try:
            trans = self.tf_buffer.lookup_transform(self.child_frame,
                                                    self.parent_frame,
                                                    rospy.Time(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        quaternionObj = trans.transform.rotation
        quaternion = (quaternionObj.x, quaternionObj.y, quaternionObj.z, quaternionObj.w)
        _, _, orientation = tr.euler_from_quaternion(quaternion)
        return (trans.transform.translation.x, trans.transform.translation.y,
                orientation, quaternionObj)

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


    def set_speed(self, msg):	#Float64 is the msg that is passed
	'''
	This method changes the speed based on the speed commanded. It also changes the lookahead distance too.

	Parameters
	----------
	msgs : Float64
	    the speed commanded

	Return
	------
	none
	'''
        self.purepursuit.speed = msg.data
        #speed = msg.data
        self.purepursuit.update_lookahead(msg.data, self.purepursuit.lookahead_min, self.purepursuit.lookahead_max, self.purepursuit.lower_threshold_v, self.purepursuit.upper_threshold_v, self.purepursuit.lookahead_gain) # [different function in 
	rospy.loginfo('speed changed to %5.2f m/s', msg.data)
        #purepursuit class]
        # have lower and upper bound for lookahead based on the speed

    def control_loop(self, event=None):
        '''
        The control loop computes the vehicle's speed and steering angle if
        path to track is set, and publishes AckermannDrive messages.

        Parameters
        ----------
        event=None : rospy.TimerEvent
            information about the event that generated this call

        Returns
        -------
        None
        '''
        #rospy.loginfo('In control_loop')
	msg = AckermannDrive(1.0, 0, 0, 0, 0)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.parent_frame
	vehicle_location_msg = PoseStamped()
	vehicle_location_msg.header.stamp = rospy.Time.now()
	vehicle_location_msg.header.frame_id = self.parent_frame
	if self.purepursuit.path is not None:
	    rospy.loginfo('there is a path')
            position = self.purepursuit.future_point()
            pose_msg.pose.position.x = position.x
            pose_msg.pose.position.y = position.y
            msg.speed = self.purepursuit.speed
            locationOfVehicle = self.get_vehicle_pose()
	    vehicle_location_msg.pose.position.x = locationOfVehicle[0]
	    vehicle_location_msg.pose.position.y = locationOfVehicle[1]
	    vehicle_location_msg.pose.orientation = locationOfVehicle[3]
	    rospy.loginfo(str(locationOfVehicle))
	    vehicle_location_msg.pose.orientation = locationOfVehicle[3] 
	    #rospy.loginfo('Vehicle - X:%5.2f Y:%5.2f', locationOfVehicle[0][0], locationOfVehicle[0][1])
	    rospy.loginfo('speed: %5.2f', msg.speed)
	    msg.steering_angle = self.purepursuit.compute_steering_angle()
        self.command_pub.publish(msg)
        self.target_pub.publish(pose_msg)
	self.draw_lookahead()
	self.draw_speed_visual()
	self.vehicle_location_pub.publish(vehicle_location_msg)

    def draw_lookahead(self):
	'''
    	Uses the vehicle location to draw a path circle around the vehicle. Publishes the shape.
    
    	Parameters
    	----------
    	None

    	Returns
    	-------
    	None
    	'''
	la_distance = self.purepursuit.lookahead 	#lookahead distance
	locationOfVehicle = self.get_vehicle_pose()	#location of vehicle (center of circle)
	lookaheadCircle = Path()			#circle of lookahead radius
	lookaheadCircle.header.stamp = rospy.Time.now()
	lookaheadCircle.header.frame_id = self.parent_frame
        theta = 0
	deltaTheta = math.pi / 16
	endTheta = 2 * math.pi + deltaTheta		#End case: 2pi
	while theta <= endTheta:
	    pose = PoseStamped()							#Position part
	    pose.header.stamp = rospy.Time.now()					#stamp
	    pose.header.frame_id = self.parent_frame					#reference
	    pose.pose.position.x = locationOfVehicle[0] + la_distance * math.cos(theta)	#x location
	    pose.pose.position.y = locationOfVehicle[1] + la_distance * math.sin(theta)	#y location
	    lookaheadCircle.poses.append(pose)						#add location to path
	    theta = theta + deltaTheta			#increment the theta
	self.lookahead_pub.publish(lookaheadCircle)

    def draw_speed_visual(self):
        '''
        Uses the vehicle location to draw a cross with side lengths equal to the meters for the meters for second
	speed.
    
        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
	#Make Path
	locationOfVehicle = self.get_vehicle_pose()     #location of vehicle (center of circle)
        speed_vis = Path()                        #circle of lookahead radius
        speed_vis.header.stamp = rospy.Time.now()
        speed_vis.header.frame_id = self.parent_frame
	speed = self.purepursuit.speed
	
	#<1,0>
	pose = PoseStamped()                                                        #Position part
        pose.header.stamp = rospy.Time.now()                                        #stamp
        pose.header.frame_id = self.parent_frame                                    #reference
        pose.pose.position.x = locationOfVehicle[0] + speed * 1 		    #x location
        pose.pose.position.y = locationOfVehicle[1] + speed * 0 		    #y location
        speed_vis.poses.append(pose)	                                            #add location to path

	#<0,0>
        pose = PoseStamped()                                                        #Position part
        pose.header.stamp = rospy.Time.now()                                        #stamp
        pose.header.frame_id = self.parent_frame                                    #reference
        pose.pose.position.x = locationOfVehicle[0] + speed * 0                     #x location
        pose.pose.position.y = locationOfVehicle[1] + speed * 0                     #y location
        speed_vis.poses.append(pose)                                                #add location to path
	
	#<-1,0>
        pose = PoseStamped()                                                        #Position part
        pose.header.stamp = rospy.Time.now()                                        #stamp
        pose.header.frame_id = self.parent_frame                                    #reference
        pose.pose.position.x = locationOfVehicle[0] + speed * -1                     #x location
        pose.pose.position.y = locationOfVehicle[1] + speed * 0                     #y location
        speed_vis.poses.append(pose)                                                #add location to path

	#<0,0>
        pose = PoseStamped()                                                        #Position part
        pose.header.stamp = rospy.Time.now()                                        #stamp
        pose.header.frame_id = self.parent_frame                                    #reference
        pose.pose.position.x = locationOfVehicle[0] + speed * 0                     #x location
        pose.pose.position.y = locationOfVehicle[1] + speed * 0                     #y location
        speed_vis.poses.append(pose)                                                #add location to path

        #<0,1>
        pose = PoseStamped()                                                        #Position part
        pose.header.stamp = rospy.Time.now()                                        #stamp
        pose.header.frame_id = self.parent_frame                                    #reference
        pose.pose.position.x = locationOfVehicle[0] + speed * 0                     #x location
        pose.pose.position.y = locationOfVehicle[1] + speed * 1                     #y location
        speed_vis.poses.append(pose)                                                #add location to path

	#<0,0>
        pose = PoseStamped()                                                        #Position part
        pose.header.stamp = rospy.Time.now()                                        #stamp
        pose.header.frame_id = self.parent_frame                                    #reference
        pose.pose.position.x = locationOfVehicle[0] + speed * 0                     #x location
        pose.pose.position.y = locationOfVehicle[1] + speed * 0                     #y location
        speed_vis.poses.append(pose)                                                #add location to path

        #<0,-1>
        pose = PoseStamped()                                                        #Position part
        pose.header.stamp = rospy.Time.now()                                        #stamp
        pose.header.frame_id = self.parent_frame                                    #reference
        pose.pose.position.x = locationOfVehicle[0] + speed * 0                     #x location
        pose.pose.position.y = locationOfVehicle[1] + speed * -1                     #y location
        speed_vis.poses.append(pose)                                                #add location to path
	
	#Publish Shape
	self.speed_visual_pub.publish(speed_vis)

if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('purepursuit', anonymous=False)
    # Create the node object
    _ = PurePursuitNode()
    # Keep the node alive
    print('About to run spin')
    rospy.spin()
    

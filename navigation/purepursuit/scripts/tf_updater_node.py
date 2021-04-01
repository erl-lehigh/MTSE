#! /usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tr
import math
import time
import tf_conversions

from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

import geometry_msgs.msg

class TFUpdaterNode(object):
	'''
	This node will receive messages for the ackermann msgs in 'speed_commad'
	
	Attributes
	----------
	current_command : AckermannDrive.ackermann_msgs.msg
	    the current Ackermann, 
	
	Methods
	-------
	
	'''

    def __init__(self):
	'''
	Initializes by setting up the publisher to change the tf and then also subscribe to the  the ackermann 'speed_command'
	'''
	#Initial Stuff
	self.rate = rospy.get_param('~rate', 1)
	self.node_name = rospy.get_name()

	#Create Subscriber
	self.command_sub = rospy.Subscriber('speed_command',
						AckermannDrive,
						self.getMessage)
	#Create AckermannDrive Message holder
	self.adMessage = AckermannDrive( 0.0, 0, 0, 0, 0)

	#Create TF
	self.tfBroadcaster = tf.TransformBroadcaster()
	
	# Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.updateVehicle)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def getMessage(self, msg):
	'''
	Get and set adMessage

	Parameters
        ----------
        msg : ackermannmsgs.msg.AckermannDrive
            car controll message
	
	Returns
	-------
	'''
	self.adMessage = msg


    def updateVehicle(self, event=None):
	'''
	Dependent on the Ackermann message it should update the location of the child frame vehicle.

	Since this method will be called in accordance to the rate, we will have the distance change be a function of the speed and steering angle

	Parameters
	----------
	event=None : rospy.TimerEvent
            information about the event that generated this call

	Return
	------
	None
	'''
	deltaMove = (1 / self.rate) * self.adMessage.speed					#use the period time the speed to have the total distance
										#traveled this cycle.
	deltaX = deltaMove * math.cos(self.adMessage.steering_angle)			#The translation ins x is cosine of steering angle
	deltaY = deltaMove * math.sin(self.adMessage.steering_angle)
	
	#transform part
	t = geometry_msgs.msg.TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "world"
	t.child_frame_id = "vehicle"
	t.transform.translation.x = deltaX
	t.transform.translation.y = deltaY
	t.transform.translation.z = 0.0
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.adMessage.steering_angle)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	#Sending Transform
	self.tfBroadcaster.sendTransform(t)

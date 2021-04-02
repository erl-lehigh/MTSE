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
	self.period = rospy.Duration(1.0 / self.rate)
	self.node_name = rospy.get_name()
	self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')
	#self.counter = -5
	#Create Subscriber
	self.command_sub = rospy.Subscriber('speed_command',
						AckermannDrive,
						self.getMessage)
	#Create AckermannDrive Message holder
	self.adMessage = AckermannDrive( 0.0, 0, 0, 0, 0)
	
	#Iterables
	##2D moventment
	self.x = 0
	self.y = 0
	##Quanterion
	self.qx = 0
	self.qy = 0
	self.qz = 0
	self.qw = 0
	##Orientation(Angle)
	self.theta = -1.0

	
	#Create TF
	self.tfBroadcaster = tf2_ros.TransformBroadcaster()
	
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
	#get location currently
	'''
	try:
            trans = self.tfBuffer.lookup_transform(self.child_frame,
                                                    self.parent_frame,
                                                    rospy.Time(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as err:
            print("Error: ", err)
	    return
        quaternionObj = trans.transform.rotation
        quaternion = (quaternionObj.x, quaternionObj.y, quaternionObj.z, quaternionObj.w)
        _, _, orientation = tr.euler_from_quaternion(quaternion)
	'''
	deltaMove = (1 / self.rate) * self.adMessage.speed					#use the period time the speed to have the total distance
	
	self.theta = self.theta + self.adMessage.steering_angle
	
	deltaX = deltaMove * math.cos(self.theta) # + orientation) + trans.transform.translation.x 		#The translation ins x is cosine of steering angle
	deltaY = deltaMove * math.sin(self.theta) # + orientation) # + trans.transform.translation.y
	
	#transform part
	t = geometry_msgs.msg.TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "world"
	t.child_frame_id = "vehicle"
	self.x = self.x + deltaX
	self.y = self.y + deltaY
	t.transform.translation.x = self.x
	t.transform.translation.y = self.y
	#self.counter = self.counter + .1
	t.transform.translation.z = 0.0
	q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta) # + orientation)
	#address qs
	t.transform.rotation.x = q[0]
	self.qx = q[0]
	t.transform.rotation.y = q[1]
	self.qy = q[1]
	t.transform.rotation.z = q[2]
	self.qz = q[2]
	t.transform.rotation.w = q[3]
	self.qw = q[3]
	
	#Sending Transform
	self.tfBroadcaster.sendTransform(t)
	rospy.loginfo("( %5.2f , %5.2f , %5.2f )", self.x, self.y, self.theta)
	
if __name__ == "__main__":
	#initialize node with rospy
	rospy.init_node('vehicle_broadcaster', anonymous=False)
	#create the node object
	_ = TFUpdaterNode()
	#Keep the node alive
	print('spinning variable speed')
	rospy.spin()

#! /usr/bin/env python
# note: copied code from the pure pursuit scripts  and modifying it for my needs
#most of the code is copied from tf_updater_node.py but also stuff from the other 2 files
#this is what runs if the arbiter decides the car needs to stop 


import rospy
import tf2_ros
import tf.transformations as tr
import math
import time
import tf_conversions

from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

import geometry_msgs.msg

class StopBehaviorNode(object):
    '''
    The node simulates the vehicle motion based on the bycicle model, and
    Ackermann speed command messages.
    Attributes
    ----------
    current_command : AckermannDrive.ackermann_msgs.msg
        the current Ackermann,
    Methods
    -------
    getMessage(msg):
        gets the tf messages and stores them for the node
    updateVehicle(event=None):
        uses the message information to update the vehicle location
    stopthecar
        update how far the vehicle is from the stop point 
        this may be helpful in determining how fast the car should come to a stop

    '''

    def __init__(self):
        '''
        Initializes by setting up the publisher      #i think there is a typo in here
        to change the tf and then also subscribe
        to the  the ackermann 'speed_command'

        added a 
        '''
        #Initial Stuff
        self.rate = rospy.get_param('~rate', 1)
        self.period = rospy.Duration(1.0 / self.rate)
        self.node_name = rospy.get_name()
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')
        #self.counter = -5

        #Create Publisher objects 
        self.stopping_command_pub = rospy.Publisher('stop_speed_command',   #this will publish the commands telling the car to slow to a stop
                                           AckermannDrive,        #idk what this message type is like
                                           queue_size=1)

        #Create Subscriber 
        self.command_sub = rospy.Subscriber('speed_command',   #suscribes to the 'spped_command' topic
                            AckermannDrive,         #message type
                            self.get_message)         #every time something is recieved, it runs get_message method
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
                                 self.update_vehicle_location)
        rospy.logdebug('[%s] Node started!', self.node_name)

    def sinusoidalSpeed(self, event=None): #the sinusoidalSpeed method is copied from variable_speed_node.py 

        '''
         Used the difference in time start and now to change the speed.
        The speed is centered at 3.35 which is the midpoint between
        the lookaway velocity bounds.
        Parameters
        ----------
        None
        Returns
        -------
        None   
        
        '''
        difTime = time.time() - self.startTime #difference in time
        thetaTime = difTime/3 #divide delta by 60 (make more smooth)
        #calculate the speed
        speed = self.base_speed + self.amplitude * math.cos(thetaTime)
        speed_msg = Float64() #initialize the message
        speed_msg.data = speed #set msg data section
        self.reference_speed_pub.publish(speed_msg)    #publish message


    def get_message(self, msg):
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

    

    def update_vehicle_location(self, event=None):
        '''
        Dependent on the Ackermann message it should update the location
        of the child frame vehicle. Since this method will be called
        in accordance to the rate, we will have the distance change
        be a function of the speed and steering angle
        Parameters
        ----------
        event=None : rospy.TimerEvent
                information about the event that generated this call
        Return
        ------
        None
        '''
        #get location current location

        deltaMove = (1 / self.rate) * self.adMessage.speed
        #use the period time and the speed to have the total distance

        self.theta = self.theta + self.adMessage.steering_angle

        deltaX = deltaMove * math.cos(self.theta)
        #The translation in x is cosine of steering angle
        deltaY = deltaMove * math.sin(self.theta)

        #transform part
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "vehicle"
        self.x = self.x + deltaX
        self.y = self.y + deltaY
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(
        0, 0, self.theta)
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
        rospy.logdebug("( %5.2f , %5.2f , %5.2f )", self.x, self.y, self.theta)

if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('stop_behavior', anonymous=False)
    # create the node object
    stop_behavior_node = StopBehaviorNode()
    # keep the node alive
    rospy.spin()
#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point

class PoseWirterNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.rate = rospy.get_param('~rate', 1)
        self.pose_sub = rospy.Subscriber('external_pose', PoseStamped, self.write_pose)
	with open("./path_locations.txt", 'w') as fp:
            fp.write('x, y, z\n')

    def write_pose(self, msg):
        c_point = msg.pose.position
        string_out = str(c_point.x) + ', ' + str(c_point.y) + ', ' + str(c_point.z) + '\n'
        with open("./path_locations.txt", 'a') as fp:
            fp.write(string_out)
        print(string_out)

if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('pose_writer', anonymous=False)
    # create the node object
    _ = PoseWirterNode()
    # keep the node alive
    rospy.spin()

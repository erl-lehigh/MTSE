#! /usr/bin/env python

'''
TODO: docstring
'''

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from purepursuit import PurePursuit


class TestPurePursuitNode(object):
    '''TODO: docstring
    '''

    def __init__(self):
        '''TODO: docstring
        '''
        self.node_name = rospy.get_name()

        # Create publishers
        self.path_pub  = rospy.Publisher('planned_path', Path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_path,
                                 oneshot=True)
        rospy.loginfo('[%s] Node started!', self.node_name)

    def publish_path(self, event=None):
        '''TODO: docstring
        '''
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "world"

        path_coords = [(-1, -1), (2, 1), (4, 0)]
        for x, y in path_coords:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"

            pose.pose.position.x = x
            pose.pose.position.y = y

            path.poses.append(pose)

        self.path_pub.publish(path)


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('test_purepursuit', anonymous=False)
    # Create the node object
    _ = TestPurePursuitNode()
    # Keep the node alive
    rospy.spin()

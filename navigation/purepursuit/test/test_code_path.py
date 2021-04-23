#! /usr/bin/env python

import unittest

import rostest

import test_future_pt

class Test_Only_Future_Point(unittest.TestCase):
    def test_pp(self):
        output = test_future_pt.dpp()
	for distance in output:
	    self.assertTrue(distance < .001) #check if point is on the line. 

if __name__ == "__main__":
    rostest.rosrun('template_ros_package', 'test_code_path', Test_Only_Future_Point)

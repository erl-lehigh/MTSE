#! /usr/bin/env python

import unittest
import rostest
import test_future_pt

class testCase(unittest.TestCase):
    def test_pp(self):
        output = test_future_pt.dpp()
	for distance in output:
	    self.assertTrue(distance < .001) #check if point is on the line. 
'''
	lookahead is:
		{3 		| Vcmd < 1.34}
		{2.24*Vcmd	| Vcmd=[1.34, 5.36]} 
		{12		| other}
'''

if __name__ == "__main__":
    rostest.rosrun('template_ros_package', 'test_code_path', testCase)

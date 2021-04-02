#! /usr/bin/env python

import unittest
import rostest
import test_purepursuit

class testCase(unittest.TestCase):
    def test_pp(self):
        output = test_purepursuit.dpp()
	self.assertEqual(output[0], 3.0)	#x = 3.0
	self.assertEqual(output[1], 3.0)	#y = 3.0
	self.assertEqual(output[2], 20.438082101802024) #radius
	self.assertEqual(output[3], 0.04892827003135632) #curv
	self.assertEqual(output[4], 3)		#speed = 3
	self.assertEqual(output[5], 0.04888928165671387) #delta
	self.assertEqual(output[6], 0.14678481009406896)  #omega
	self.assertEqual(output[7], 6.720000000000001)	#lookahead

	future_pt = output[8]
	path = output[9]
	distance = path.distance(future_pt) #if the point is on the path the distance should be really small.
	self.assertTrue(distance < .001) #check if point is on the line. 
'''
	lookahead is:
		{3 		| Vcmd < 1.34}
		{2.24*Vcmd	| Vcmd=[1.34, 5.36]} 
		{12		| other}
'''

if __name__ == "__main__":
    rostest.rosrun('template_ros_package', 'test_code', testCase)

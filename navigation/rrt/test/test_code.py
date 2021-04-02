#! /usr/bin/env python

import unittest
import rostest

class testCase(unittest.TestCase):
    def starting_point(self):
        pass
    def end_point(self):
        pass
    def check_collison(self):
        pass

if __name__ == "__main__":
    rostest.rosrun('rrt', 'test_code', testCase)
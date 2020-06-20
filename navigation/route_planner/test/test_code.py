#! /usr/bin/env python

import unittest
import rostest

class testCase(unittest.TestCase):
    def NAME_OF_THE_TEST(self):
        pass

if __name__ == "__main__":
    rostest.rosrun('template_ros_package', 'test_code', testCase)

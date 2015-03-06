#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_lk_tracker_publish'

import sys 
import time
import unittest

import rospy
import rostest

class TestLKTrackerPublish(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestLKTrackerPublish, sys.argv)
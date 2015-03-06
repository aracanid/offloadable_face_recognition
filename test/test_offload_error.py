#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_offloading_error'

import sys 
import time
import unittest

import rospy
import rostest

class TestOffloadError(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestOffloadError, sys.argv)
#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_offload_to_local'

import sys 
import time
import unittest

import rospy
import rostest

class TestOffloadToLocal(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestOffloadToLocal, sys.argv)
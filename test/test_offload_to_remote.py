#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_offload_to_remote'

import sys 
import time
import unittest

import rospy
import rostest

class TestOffloadToRemote(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestOffloadToRemote, sys.argv)
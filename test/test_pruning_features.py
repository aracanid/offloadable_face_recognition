#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_pruning_features'

import sys 
import time
import unittest

import rospy
import rostest

class TestPruningFeatures(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPruningFeatures, sys.argv)
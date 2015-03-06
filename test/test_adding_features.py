#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_adding_features'

import sys 
import time
import unittest

import rospy
import rostest

class TestAddingFeatures(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestAddingFeatures, sys.argv)
#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_face_detection'

import sys 
import time
import unittest

import rospy
import rostest

class TestFaceDetection(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestAddingFeatures, sys.argv)
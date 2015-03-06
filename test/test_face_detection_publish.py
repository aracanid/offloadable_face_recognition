#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_face_detection_publish'

import sys 
import time
import unittest

import rospy
import rostest

class TestFaceDetectionPublish(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestFaceDetectionPublish, sys.argv)
#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_pre_processing_publish'

import sys 
import time
import unittest

import rospy
import rostest

class TestPreProcessingPublish(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPreProcessingPublish, sys.argv)
#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_post_processing_output'

import sys 
import time
import unittest

import rospy
import rostest

class TestPostProcessingOutput(unitttest.TestCase):
	pass

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPostProcessingOutput, sys.argv)
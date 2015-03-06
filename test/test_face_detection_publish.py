#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_face_detection_publish'

import sys 
import time
import unittest

import rospy
import rostest

from offloadable_face_recognition.msg import FaceBox

class TestFaceDetectionPublish(unitttest.TestCase):

	def __init__(self, *args):
		super(TestFaceDetectionPublish, self).__init__.(*args)
		self.success = False

	def callback(self, data):
		if data:
			self.success = True

	def test_face_detection_publish(self):
		rospy.Subscriber("face_box_coordinates", FaceBox, self.callback)
		rospy.init_node(NAME,annonymous=True)
		timeout = time.time() + 10.0*1000 #10 seconds

		while not rospy.is_shutdown() and not self.success and time.time() < timeout:
			time.sleep(0.1)
		self.assert_(self.success, str(self.success))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestFaceDetectionPublish, sys.argv)
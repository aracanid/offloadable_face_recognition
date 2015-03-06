#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_pre_processing_publish'

import sys 
import time
import unittest
import rospy
import rostest
from sensor_msgs.msg import Image



class TestPreProcessingPublish(unitttest.TestCase):
	def __init__(self, *args):
		super(TestPreProcessingPublish, self).__init__.(*args)
		self.success = False

	def callback(self, data):
		if data:
			self.success = True

	def test_pre_processing_publish(self):
		rospy.Subscriber("pre_processed_image", Image, self.callback)
		rospy.init_node(NAME,annonymous=True)
		timeout = time.time() + 10.0*1000 #10 seconds

		while not rospy.is_shutdown() and not self.success and time.time() < timeout:
			time.sleep(0.1)
		self.assert_(self.success, str(self.success))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPreProcessingPublish, sys.argv)
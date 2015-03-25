#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_face_detection_publish'

import sys 
import time
import rospy
import rostest
import unittest
from offloadable_face_recognition.msg import FaceBox
from offloadable_face_recognition.msg import OffloadCommand, SchedulerCommand

class TestFaceDetectionPublish(unittest.TestCase):

	def __init__(self, *args):
		super(TestFaceDetectionPublish, self).__init__(*args)
		self.success = False
		self.scheduler_pub = None
		rospy.init_node(NAME)

	def callback_regular(self, data):
		if data:
			if data.x > 0 and data.y > 0:
				self.success = True

	def callback_unsubscribed(self, data):
		if data:
			self.success = False

	def unsubscribe_face_detector(self):
		scheduler_command = SchedulerCommand()
		scheduler_command.node_name = "face_detector" 
		scheduler_command.offload = True
		self.scheduler_pub.publish(scheduler_command)

	def test_face_detection_publish(self):
		self.success = False
		rospy.Subscriber("face_box_coordinates", FaceBox, self.callback_regular, 1)
		timeout = time.time() + 10.0*1000 #10 seconds

		while not rospy.is_shutdown() and not self.success and time.time() < timeout:
			time.sleep(0.1)
		self.assertTrue(self.success, "Successfully published face coordinates! - " + str(self.success))

	def test_face_detection_unsubscribe_and_publish(self):
		self.success = True
		self.scheduler_pub = rospy.Publisher("scheduler_commands", SchedulerCommand, 1)
		self.unsubscribe_face_detector()
		rospy.Subscriber("face_box_coordinates", FaceBox, self.callback_unsubscribed, 1)
		timeout = time.time() + 10.0*1000 #10 seconds

		while not rospy.is_shutdown() and not self.success and time.time() < timeout:
			time.sleep(0.1)
		self.assertTrue(self.success, "Successfully offloaded face tracker! - " + str(self.success))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestFaceDetectionPublish, sys.argv)

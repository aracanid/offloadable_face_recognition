#!/usr/bin/env python

PKG = 'offloadable_face_recognition'
NAME = 'test_lk_tracker_publish'

import sys 
import time
import unittest
import threading
from offloadable_face_recognition.msg import FaceBox,CoordinatesList, SchedulerCommand
import rospy
import rostest


class TestLKTrackerPublish(unittest.TestCase):

	def __init__(self, *args):
		super(TestLKTrackerPublish, self).__init__(*args)
		self.success = False
		self.scheduler_pub = None
		rospy.init_node(NAME)

	def callback_regular(self, data):
		if data:
			if len(data.coordinates) > 0:
				self.success = True

	def callback_unsubscribed(self, data):
		if data:
			self.success = False

	def unsubscribe_lk_tracker(self):
		scheduler_command = SchedulerCommand()
		scheduler_command.node_name = "face_tracker" 
		scheduler_command.offload = True
		self.scheduler_pub.publish(scheduler_command)

	def test_lk_tracker_publish(self):
		self.success = False
		rospy.Subscriber("feature_coordinates_output", CoordinatesList, self.callback_regular, 1)

		timeout = time.time() + 10.0*1000 #10 seconds

		while not rospy.is_shutdown() and not self.success and time.time() < timeout:
			time.sleep(0.1)
		self.assertTrue(self.success, "Successfully received lk tracker feature coordinates list! - " + str(self.success))

	def test_lk_trackker_unsubscribe_and_publish(self):
		self.success = True
		self.scheduler_pub = rospy.Publisher("scheduler_commands", SchedulerCommand, 1)
		self.unsubscribe_lk_tracker()
		rospy.Subscriber("feature_coordinates_output", CoordinatesList, self.callback_unsubscribed, 1)

		timeout = time.time() + 10.0*1000 #10 seconds

		while not rospy.is_shutdown() and not self.success and time.time() < timeout:
			time.sleep(0.1)
		self.assertTrue(self.success, "Successfully offloaded face tracker! - " + str(self.success))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestLKTrackerPublish, sys.argv)
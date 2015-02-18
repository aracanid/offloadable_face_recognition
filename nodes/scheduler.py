#!/usr/bin/env python
import roslib
import rospy
import cv2
import sys
import numpy as np
import psutil as ps
import threading

from offloadable_face_recognition.msg import OffloadCommand, SchedulerCommand
from cv_bridge import CvBridge, CvBridgeError

class Scheduler:

	def __init__(self, node_name):

		node_name = rospy.get_param("~node_name", "node_name")
		print node_name
		rospy.init_node(node_name)

		print "Initialising " + node_name

		self.MANUAL_OFFLOAD_COMMANDS = "manual_offload_commands"
		self.SCHEDULER_COMMANDS = "scheduler_commands"

		self.queue_size = 1

		self.offloading_command_sub = rospy.Subscriber(self.MANUAL_OFFLOAD_COMMANDS, OffloadCommand, self.offloading_command_listener, queue_size=self.queue_size)
		self.scheduler_pub = rospy.Publisher(self.SCHEDULER_COMMANDS, SchedulerCommand, queue_size=self.queue_size)

		self.LOW_CPU_USAGE_THRESHOLD = 25.0
		self.MID_CPU_USAGE_THRESHOLD = 50.0
		self.HIGH_CPU_USAGE_THRESHOLD = 75.0
		self.OFFLOAD_TO_PC = True
		self.OFFLOAD_TO_RPI = False
		self.cpu_count = 1

		self.rpi_pre_processing_node = "rpi_pre_processing_node"
		self.rpi_face_detection_node = "rpi_face_detection_node"
		self.rpi_lk_tracker_node = "rpi_lk_tracker_node"

		self.pc_pre_processing_node = "pc_pre_processing_node"
		self.pc_face_detection_node = "pc_face_detection_node"
		self.pc_lk_tracker_node = "pc_lk_tracker_node"

		self.pp_offloaded = False
		self.fd_offloaded = False
		self.lk_offloaded = False

		self.isAutomatic = True
		self.percentage = 0

		self.offload_command_lock = threading.Lock()

		self.rate = rospy.Rate(0.5) # 20hz



		#spin the node
		self.offloading_scheduler()

	def offloading_command_listener(self, command):
		with self.offload_command_lock:
			self.isAutomatic = command.type
			self.percentage = command.percentage

	def offloading_scheduler(self):
		while True:

			cpu_usage = self.get_cpu_usage()

			if cpu_usage >= self.HIGH_CPU_USAGE_THRESHOLD and not self.lk_offloaded:
				self.offload_node(self.pc_lk_tracker_node, self.OFFLOAD_TO_PC)
				self.lk_offloaded = True
				print "offloaded lk"

			if cpu_usage >= self.MID_CPU_USAGE_THRESHOLD and not self.fd_offloaded:
				self.offload_node(self.pc_face_detection_node, self.OFFLOAD_TO_PC)
				print "offloaded fd"

			if cpu_usage >= self.LOW_CPU_USAGE_THRESHOLD and not self.pp_offloaded:
				self.offload_node(pc_pre_processing_node, self.OFFLOAD_TO_PC)
				self.pp_offloaded = True
				print "offloaded pp"

			if cpu_usage < self.HIGH_CPU_USAGE_THRESHOLD and self.lk_offloaded:
				self.offload_node(self.rpi_lk_tracker_node, self.OFFLOAD_TO_RPI)
				self.lk_offloaded = False
				print "unloaded lk"

			if cpu_usage < self.MID_CPU_USAGE_THRESHOLD and self.fd_offloaded:
				self.offload_node(self.rpi_face_detection_node, self.OFFLOAD_TO_RPI)
				self.fd_offloaded = False
				print "unloaded fd"

			if cpu_usage < self.LOW_CPU_USAGE_THRESHOLD and self.pp_offloaded:
				self.offload_node(self.rpi_pre_processing_node, self.OFFLOAD_TO_RPI)
				self.pp_offloaded = False
				print "unloaded pp"
				
			#elif cpu_usage < self.LOW_CPU_USAGE_THRESHOLD:


			####
			# TODO: ADD WIFI SIGNAL STRENGTH CHECKER
			###

			# Fill the remainder of the frequency with a wait to prevent excessive spinning
			self.rate.sleep()

	def get_cpu_usage(self):
		with self.offload_command_lock:
			isAutomatic = self.isAutomatic

		if self.isAutomatic:
			cpu_usage_sum = 0

			# If there are multiple CPUs then we should take the average between these
			if self.cpu_count > 1:
				cpu_usage = ps.cpu_percent(percpu=True)
				for cpu in cpu_usage:
					cpu_usage_sum = cpu_usage_sum + cpu
				cpu_usage = cpu_usage_sum/self.cpu_count
			else:
				cpu_usage = ps.cpu_percent()
				print str(cpu_usage)

			print "actual cpu output " + str(cpu_usage) #remove
		else:
			with self.offload_command_lock:
				cpu_usage = self.percentage
			print "manual cpu output " + str(cpu_usage) #remove

			return cpu_usage

	def offload_node(self, node_name, destination):
		try:
			scheduler_command = SchedulerCommand()
			scheduler_command.node_name = node_name
			scheduler_command.offload = destination
			self.scheduler_pub.publish(scheduler_command)
		except CvBridgeError, e:
			print e


def main(args):
	try:   
		# Fire up the node.
		SC = Scheduler("scheduler")
		# Spin so our services will work
		print "Node started..."
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down vision node."
		cv.DestroyAllWindows()

if __name__ == '__main__':

	main(sys.argv)
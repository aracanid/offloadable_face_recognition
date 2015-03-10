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

		rospy.init_node(node_name)
		print "Initialising " + node_name

		self.MANUAL_OFFLOAD_COMMANDS = "manual_offload_commands"
		self.SCHEDULER_COMMANDS = "scheduler_commands"
		self.NODE_REMOTE = True
		self.NODE_LOCAL = False

		self.queue_size = 10

		self.offloading_command_sub = rospy.Subscriber(self.MANUAL_OFFLOAD_COMMANDS, OffloadCommand, self.offloading_command_listener, queue_size=self.queue_size)
		self.scheduler_pub = rospy.Publisher(self.SCHEDULER_COMMANDS, SchedulerCommand, queue_size=self.queue_size)

		self.LOW_CPU_USAGE_THRESHOLD = 25.0
		self.MID_CPU_USAGE_THRESHOLD = 50.0
		self.HIGH_CPU_USAGE_THRESHOLD = 75.0
		self.UNSUBSCRIBE = True
		self.SUBSCRIBE = False
		self.cpu_count = 1

		self.rpi_pre_processing_node = "rpi_pre_processing_node"
		self.rpi_face_detection_node = "rpi_face_detection_node"
		self.rpi_lk_tracker_node = "rpi_lk_tracker_node"

		self.pc_pre_processing_node = "pc_pre_processing_node"
		self.pc_face_detection_node = "pc_face_detection_node"
		self.pc_lk_tracker_node = "pc_lk_tracker_node"

		self.isAutomatic = True
		self.percentage = 0
		self.cpu_usage_lock = threading.Lock()

		self.offload_command_lock = threading.Lock()

		self.rate = rospy.Rate(0.5) # 20hz
		self.init_rate = rospy.Rate(0.1)

		self.initialise_nodes(True)

		self.init_rate.sleep()

		self.system_on = True

		self.lk_offloaded = False
		self.fd_offloaded = False
		self.pp_offloaded = False

		self.cpu_usage_thread = threading.Thread(target = self.offloading_scheduler)
		self.cpu_usage_thread.start()
		#spin the node
		self.offloading_scheduler()

	def offloading_command_listener(self, command):
		with self.offload_command_lock:
			self.isAutomatic = command.type
			self.percentage = command.percentage

	def set_system_on(self, state):
		with self.system_on_lock:
			self.system_on = state

	def get_system_on(self):
		return self.system_on

	# Separate cpu_usage into another thread
	def offloading_scheduler(self):
		while self.get_system_on():
			try:
				cpu_usage = self.get_cpu_usage()

				if cpu_usage >= self.HIGH_CPU_USAGE_THRESHOLD and not self.lk_offloaded:

					self.offload_node(self.rpi_lk_tracker_node, self.UNSUBSCRIBE)
					self.offload_node(self.pc_lk_tracker_node, self.SUBSCRIBE)
					self.lk_offloaded = self.NODE_REMOTE

				if cpu_usage >= self.MID_CPU_USAGE_THRESHOLD and not self.fd_offloaded:

					self.offload_node(self.rpi_face_detection_node, self.UNSUBSCRIBE)
					self.offload_node(self.pc_face_detection_node, self.SUBSCRIBE)
					self.fd_offloaded = self.NODE_REMOTE

				if cpu_usage >= self.LOW_CPU_USAGE_THRESHOLD and not self.pp_offloaded:

					self.offload_node(self.rpi_pre_processing_node, self.UNSUBSCRIBE)
					self.offload_node(self.pc_pre_processing_node, self.SUBSCRIBE)
					self.pp_offloaded = self.NODE_REMOTE

				if cpu_usage < self.HIGH_CPU_USAGE_THRESHOLD and self.lk_offloaded:
					self.offload_node(self.pc_lk_tracker_node, self.UNSUBSCRIBE)
					self.offload_node(self.rpi_lk_tracker_node, self.SUBSCRIBE)
					self.lk_offloaded = self.NODE_LOCAL

				if cpu_usage < self.MID_CPU_USAGE_THRESHOLD and self.fd_offloaded:
					self.offload_node(self.pc_face_detection_node, self.UNSUBSCRIBE)
					self.offload_node(self.rpi_face_detection_node, self.SUBSCRIBE)
					self.fd_offloaded = self.NODE_LOCAL

				if cpu_usage < self.LOW_CPU_USAGE_THRESHOLD and self.pp_offloaded:
					self.offload_node(self.pc_pre_processing_node, self.UNSUBSCRIBE)
					self.offload_node(self.rpi_pre_processing_node, self.SUBSCRIBE)
					self.pp_offloaded = self.NODE_LOCAL
					
				#elif cpu_usage < self.LOW_CPU_USAGE_THRESHOLD:


				####
				# TODO: ADD WIFI SIGNAL STRENGTH CHECKER
				###

				# Fill the remainder of the frequency with a wait to prevent excessive spinning
				self.rate.sleep()

			except KeyboardInterrupt:
				print "Scheduler shutting down..."
				self.set_system_on(False)
				raise

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
		else:
			with self.offload_command_lock:
				cpu_usage = self.percentage

		
		return cpu_usage

	def offload_node(self, node_name, destination):
		try:
			scheduler_command = SchedulerCommand()
			scheduler_command.node_name = node_name
			scheduler_command.offload = destination
			self.scheduler_pub.publish(scheduler_command)
			self.rate.sleep()
		except OffloadingError, e:
			print "Could not offload node " + node_name + "\n" + "-----\n" + e

	def initialise_nodes(self, isLocal):
		self.rate.sleep()
		if isLocal == True:
			self.set_nodes_status(self.NODE_LOCAL)
			self.offload_node(self.rpi_pre_processing_node, self.pp_offloaded)
			self.offload_node(self.rpi_lk_tracker_node, self.lk_offloaded)
			self.offload_node(self.rpi_face_detection_node, self.fd_offloaded)
		else:
			self.set_nodes_status(self.NODE_REMOTE)
			self.offload_node(self.pc_pre_processing_node, self.pp_offloaded)
			self.offload_node(self.pc_face_detection_node, self.fd_offloaded)
			self.offload_node(self.pc_lk_tracker_node, self.lk_offloaded)

	def set_nodes_status(self, state):
			self.lk_offloaded = state
			self.fd_offloaded = state
			self.pp_offloaded = state

def main(args):
	try:   
		# Fire up the node.
		SC = Scheduler("scheduler")
		# Spin so our services will work
		print "Node started..."
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down " + SC.node_name

if __name__ == '__main__':

	main(sys.argv)
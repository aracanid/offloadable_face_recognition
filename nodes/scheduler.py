import roslib
import rospy
import cv2
import sys
import numpy as np
import psutil as ps

from  offloadable_fr_node import Offloadable_FR_Node
from offloadable_face_recognition.msg import OffloadCommand
from cv_bridge import CvBridge, CvBridgeError

class Scheduler(Offloadable_FR_Node):

	def __init__(self, node_name):

		print "Initialising " + node_name

		Offloadable_FR_Node.__init__(self, node_name)

		self.offloading_command_sub = rospy.Subscriber("scheduler_commands", OffloadCommand, offloading_command_update, queue_size=self.queue_size)

		self.MAX_LOW_CPU_USAGE = 25.0
		self.MAX_MID_CPU_USAGE = 50.0
		self.MAX_HIGH_CPU_USAGE = 75.0
		self.cpu_count = ps.cpu_count()

		# Preprocessing topics
		self.pp_topic_one = "pp_topic_one"
		self.pp_topic_two = "pp_topic_two"
		self.pp_topics = [self.pp_topic_one, self.pp_topic_two]

		# Face dectection topics
		self.fd_topic_one = "fd_topic_one"
		self.fd_topic_two = "fd_topic_two"
		self.fd_topics = [self.fd_topic_one, self.fd_topic_two]

		# Lucas-Kande tracking topics
		self.lk_topic_one = "lk_topic_one"
		self.lk_topic_one = "lk_topic_two"
		self.lk_topics = [self.lk_topic_one, self.lk_topic_two]

		self.pp_offloaded = False
		self.fd_offloaded = False
		self.lk_offloaded = False

		# Multiplexer identifiers
		self.mux_pp = "mux_pp"
		self.mux_fd = "mux_fd"
		self.mux_lk = "mux_lk"

		self.isAutomatic = True
		self.percentage = 0

		self.offload_command_lock = threading.Lock()

	def offloading_command_update(self, command):
		with offload_command_lock:
			self.isAutomatic = command.isAutomatic
			self.percentage = command.percentage

	def offloading_scheduler(self):
		with offload_command_lock:
			isAutomatic = self.isAutomatic

		if isAutomatic:
			cpu_usage_sum = 0

			# If there are multiple CPUs then we should take the average between these
			if self.cpu_count > 1:
				cpu_usage = ps.cpu_percent(percpu=True)
				for cpu in cpu_usage:
					cpu_usage_sum = cpu_usage_sum + cpu_usage
				cpu_usage = cpu_usage_sum/self.cpu_count
			else:
				cpu_usage = ps.cpu_percent()
				print str(cpu_usage)
		else:
			with offload_command_lock:
				cpu_usage = self.percentage

		print "actual cpu output" + cpu_usage #remove

		cpu_usage = 51

		if cpu_usage >= self.MAX_HIGH_CPU_USAGE and not self.lk_offloaded:
			offload_to_node(self.lk_topics[1], self.mux_lk)
			self.lk_offloaded = True
			print "offloaded lk"
		elif cpu_usage >= self.MAX_MID_CPU_USAGE and not self.fd_offloaded:
			offload_to_node(self.fd_topics[1], self.mux_fd)
			self.fd_offloaded = True
			print "offloaded fd"
		elif cpu_usage >= self.MAX_LOW_CPU_USAGE and not self.pp_offloaded:
			offload_to_node(self.pp_topics[1], self.mux_pp)
			self.pp_offloaded = True
			print "offloaded pp"
		#elif cpu_usage < self.MAX_LOW_CPU_USAGE:


		####
		# TODO: ADD WIFI SIGNAL STRENGTH CHECKER
		###

		# Fill the remainder of the frequency with a wait to prevent excessive spinning
		self.rate.sleep()

	def offload_to_node(self, target_topic, target_mux):
		offload_node = rospy.ServiceProxy('mux_select', (target_mux).select)
			try:
				prev_topic = offload_node(target_topic)
				print "Offloaded to node" + str(prev_topic)
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))
		return prev_topic

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
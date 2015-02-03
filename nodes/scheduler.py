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

		image_sub = rospy.Subscriber("scheduler_commands", OffloadCommand, offloading_command_update, queue_size=self.queue_size)

		self.MAX_LOW_CPU_USAGE = 25.0
		self.MAX_MID_CPU_USAGE = 50.0
		self.MAX_HIGH_CPU_USAGE = 75.0
		self.cpu_count = ps.cpu_count()

		self.pp_topic_one = "pp_topic_one"
		self.pp_topic_two = "pp_topic_two"
		self.fd_topic_one = "fd_topic_one"
		self.fd_topic_two = "fd_topic_two"
		self.lk_topic_one = "lk_topic_one"
		self.lk_topic_one = "lk_topic_two"

		self.current_pp_topic = "pp_topic_one"
		self.current_fd_topic = "fd_topic_one"
		self.current_lk_topic = "lk_topic_one"

		self.prev_pp_topic = "pp_topic_two"
		self.prev_fd_topic = "fd_topic_two"
		self.prev_lk_topic = "lk_topic_two"

		self.mux_pp = "mux_pp"
		self.mux_fd = "mux_fd"
		self.mux_lk = "mux_lk"

		self.is_auto = True
		self.percentage = 0

		self.offload_command_lock = threading.Lock()

		self.offloading_command_sub = rospy.Subscriber(self.offloading_command_sub, OffloadCommand, self.offloading_command, queue_size=self.queue_size)


	def offloading_command_update(self, command):
		with offload_command_lock:
			self.is_auto = command.is_auto
			self.percentage = command.percentage

	def offloading_scheduler(self):
		with offload_command_lock:
			is_auto = self.is_auto

		if is_auto:
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

		cpu_usage = 50 #remove

		if cpu_usage >= self.MAX_HIGH_CPU_USAGE:
			self.current_lk_topic = offload_to_node(self.prev_lk_topic, self.mux_lk)
			print "offloaded lk"
		elif cpu_usage >= self.MAX_MID_CPU_USAGE:
			self.current_fd_topic = offload_to_node(self.prev_fd_topic, self.mux_fd)
			print "offloaded fd"
		elif cpu_usage >= self.MAX_LOW_CPU_USAGE:
			self.current_pp_topic = offload_to_node(self.prev_pp_topic, self.mux_pp)
			print "offloaded pp"
		#elif cpu_usage < self.MAX_LOW_CPU_USAGE:

		# Fill the remainder of the frequency with a wait to prevent excessive spinning
		self.rate.sleep()

	def offload_to_node(self, target_topic, target_mux):
		offload_node = rospy.ServiceProxy('mux_select', target_mux/select)
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
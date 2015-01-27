import roslib
import rospy
import cv2
import sys
import numpy as np
import psutil as ps

from  offloadable_fr_node import Offloadable_FR_Node
from cv_bridge import CvBridge, CvBridgeError

class Scheduler(Offloadable_FR_Node):

	def __init__(self, node_name):

		print "Initialising " + node_name

		Offloadable_FR_Node.__init__(self, node_name)

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

		self.prev_pp_topic = "pp_topic_two"
		self.prev_fd_topic = "fd_topic_two"
		self.prev_lk_topic = "lk_topic_two"

		self.mux_pp = "mux_pp"
		self.mux_fd = "mux_fd"
		self.mux_lk = "mux_lk"


	def offloading_scheduler(self):

		cpu_usage_sum = 0

		# If there are multiple CPUs then we should take the average between these
		if self.cpu_count > 1:
			cpu_usage = ps.cpu_percent(percpu=True)
			for cpu in cpu_usage:
				cpu_usage_sum = cpu_usage_sum + cpu_usage
			cpu_usage = cpu_usage_sum/self.cpu_count
		else:
			cpu_usage = ps.cpu_percent()

		if cpu_usage >= self.MAX_HIGH_CPU_USAGE:
			self.prev_lk_topic = offload_to_node(self.prev_lk_topic, self.mux_lk)
		elif cpu_usage >= self.MAX_MID_CPU_USAGE:
			self.prev_fd_topic = offload_to_node(self.prev_fd_topic, self.mux_fd)
		elif cpu_usage >= self.MAX_LOW_CPU_USAGE:
			self.prev_pp_topic = offload_to_node(self.prev_pp_topic, self.mux_pp)
		#elif cpu_usage < self.MAX_LOW_CPU_USAGE:

		# Fill the remainder of the frequency with a wait to prevent excessive spinning
		self.rate.sleep()

	def offload_to_node(self, target_topic, target_mux):
		offload_node = rospy.ServiceProxy('mux_select', target_mux/select)

		if cpu_usage >= cpu_threshold:
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
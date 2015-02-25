#!/usr/bin/env python

import roslib
import rospy
import cv2
import sys
import numpy as np

from  offloadable_fr_node import Offloadable_FR_Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from offloadable_face_recognition.msg import SchedulerCommand

class Image_Pre_Processing(Offloadable_FR_Node):

	def __init__(self, node_name):

		print "Initialising " + node_name

		Offloadable_FR_Node.__init__(self, node_name)

		# Intemediary images
		self.grey = None
		self.pre_processed_image = None
		self.marker_image = None

		# Subscribe to the raw camera image topic and set the image processing callback to self.pre_processing()
		#self.image_sub = rospy.Subscriber(self.input_rgb_image, Image, self.pre_processing, queue_size=self.queue_size)
		# # A publisher to output the greyscale processed image
		#self.pre_processed_image_pub = rospy.Publisher(self.pre_processed_output_image, Image, queue_size=self.queue_size)
		# # A subscriber for receiving scheduler commands
		# self.scheduler_sub = rospy.Subscriber(self.scheduler_commands, SchedulerCommand, self.scheduler_listener, queue_size=self.queue_size)
		self.image_sub = None
		self.pre_processed_image_pub = None 
		self.scheduler_sub = rospy.Subscriber(self.scheduler_commands, SchedulerCommand, self.scheduler_listener, queue_size=self.queue_size)
		
		self.r = rospy.Rate(10)

	def pre_processing(self, ros_image):

		if self.is_offloaded == False:
			
			# Convert the ROS Image to Opencv format using the convert_img_to_cv() helper function
			cv_image = self.convert_img_to_cv(ros_image)

			if self.pre_processed_image is None:
				self.pre_processed_image = np.zeros(cv_image.shape, np.uint8)

			# Create a single channel greyscale image
			if self.grey is None:  
				(im_width, im_height, im_depth) = cv_image.shape
				self.grey = np.zeros((im_width,im_height,1), np.uint8)

			# Convert input image from color to greyscale
			self.grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

			# Equalize the histogram to reduce lighting effects 
			self.grey = cv2.equalizeHist(self.grey, self.grey)

			# Convert back to ROS Image from Opencv formatf
			pre_processed_image = self.convert_cv_to_img(self.grey)

			try:
				self.pre_processed_image_pub.publish(pre_processed_image)
			except CvBridgeError, e:
				print e

	def unsubscribe_node(self):
		# Function to unsubscribe a node from its topics and stop publishing data
		self.is_offloaded = True
		self.image_sub.unregister()
		self.pre_processed_image_pub = None

	def resubscribe_node(self):
		# Function to resubscribe and republish the nodes data
		self.image_sub = rospy.Subscriber(self.input_rgb_image, Image, self.pre_processing, queue_size=self.queue_size)
		self.pre_processed_image_pub = rospy.Publisher(self.pre_processed_output_image, Image, queue_size=self.queue_size)
		
		self.is_offloaded = False




def main(args):
	try:   
		PP = Image_Pre_Processing("pre_processing_node")
		print "Node started..."
		rospy.spin()
	except KeyboardInterrupt:
		print "ERROR: could not start node \n" + e

if __name__ == '__main__':
	main(sys.argv)

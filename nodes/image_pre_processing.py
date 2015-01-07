#!/usr/bin/env python
import roslib
import rospy
import cv2
import sys
from sensor_msgs.msg import Image, CameraInfo
from  offloadable_fr_node import Offloadable_FR_Node
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Image_Pre_Processing(Offloadable_FR_Node):

	def __init__(self, node_name):

		print "initialising pre_procesing_node"
		Offloadable_FR_Node.__init__(self, node_name)

		self.grey = None
		self.image_size = None
		self.image = None
		self.pre_processed_image = None
		
		# A publisher to output the processed image back to a ROS topic
		self.pre_processed_image_pub = rospy.Publisher("pre_processed_image", Image, queue_size=self.queue_size) ###change

		# Wait until the image topics are ready before starting
		rospy.wait_for_message(self.input_rgb_image, Image)

		# Subscribe to the raw camera image topic and set the image processing callback
		image_sub = rospy.Subscriber(self.input_rgb_image, Image, self.pre_processing, queue_size=self.queue_size)

	def pre_processing(self, ros_image):

		# Convert the raw image to Opencv format using the convert_img_to_cv() helper function
		cv_image_array = Offloadable_FR_Node.convert_img_to_cv(self, ros_image)
		cv_image = cv2.cv.fromarray(cv_image_array)
		
		print "converted to cv2 image"
		
		# Create a few images we will use for display
		if not self.image:
			self.image_size = cv2.cv.GetSize(cv_image)
			self.image = cv2.cv.CreateImage(self.image_size, 8, 3)
			self.pre_processed_image = cv2.cv.CreateImage(self.image_size, 8, 3)

		if self.grey is None:
			# Allocate temporary images     
			self.grey = cv2.cv.CreateImage(self.image_size, 8, 1)

		# Convert color input image to grayscale 
		self.grey = cv2.cvtColor(cv_image_array, cv2.COLOR_BGR2GRAY)

		# Equalize the histogram to reduce lighting effects. 
		self.grey = cv2.equalizeHist(self.grey, self.grey)
		
		Offloadable_FR_Node.convert_cv_to_img(self, self.grey)

		print "before sending"
		# Publish the display image back to ROS 
		try:
			self.pre_processed_image_pub.publish(pre_processed_image)
			print "image sent"
		except CvBridgeError, e:
			print e

def main(args):
	try:   
		# Fire up the node.
		PP = Image_Pre_Processing("ev3_image_pre_processing")
		print "starting pre_processing_node"
		# Spin so our services will work
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down vision node."
		cv2.DestroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
#!/usr/bin/env python

import roslib
import rospy
import cv2
import sys
import numpy as np

from offloadable_fr_node import Offloadable_FR_Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from offloadable_face_recognition.msg import FaceBox, SchedulerCommand

class Face_Detector(Offloadable_FR_Node):

	def __init__(self, node_name):

		print "Initialising " + node_name

		Offloadable_FR_Node.__init__(self, node_name)

		# Haar cascade constants
		self.MIN_SIZE = (20, 20)
		self.IMAGE_SCALE = 2
		self.HAAR_SCALE = 1.5
		self.MIN_NEIGHBORS = 1
		self.HAAR_FLAGS = cv2.cv.CV_HAAR_DO_CANNY_PRUNING

		# Harr cascade classifiers
		self.cascade_frontal_alt = rospy.get_param("~cascade_frontal_alt", "")
		self.cascade_frontal_alt2 = rospy.get_param("~cascade_frontal_alt2", "")
		self.cascade_profile = rospy.get_param("~cascade_profile", "")
		
		self.cascade_frontal_alt = cv2.CascadeClassifier(self.cascade_frontal_alt)
		self.cascade_frontal_alt2 = cv2.CascadeClassifier(self.cascade_frontal_alt2)
		self.cascade_profile = cv2.CascadeClassifier(self.cascade_profile)

		# Intemediary images
		self.small_image = None

		# A publisher to output face coordinates and size 
		self.output_face_box_pub = rospy.Publisher(self.face_box_coordinates, FaceBox, queue_size=self.queue_size)  ###need to change this so that it sends an array

		# # A publisher to output the image used for marker information on top of the output image
		# self.marker_image_pub = rospy.Publisher(self.marker_image_output, Image, queue_size=self.queue_size)

		# A publisher to output the final image and display it
		self.output_image_pub = rospy.Publisher(self.output_image, Image, queue_size=self.queue_size)  ###need to change this so that it sends an array

		# A publisher to output the frame in which the face was detected
		self.face_detect_output_image_pub = rospy.Publisher(self.face_detect_output_image, Image,queue_size=self.queue_size)

		# Subscribe to the preprocessed image output and set the detect_face as the callback
		self.image_sub = rospy.Subscriber(self.pre_processed_output_image, Image, self.detect_face, queue_size=self.queue_size)

	def detect_face(self, ros_image):

		# Convert the ROS Image to Opencv format using the convert_img_to_cv() helper function
		cv_image = self.convert_img_to_cv(ros_image)

		im_width, im_height = cv_image.shape

		# Create a single scaled down image
		if self.small_image is None:
			self.small_image = np.zeros((im_width/self.IMAGE_SCALE, im_height/self.IMAGE_SCALE, 1), np.uint8)
			# self.marker_image = np.zeros((im_width, im_height, 3), np.uint8)

		# Scale input image for faster processing using the scaled image
		self.small_image = cv2.resize(cv_image, (im_width/self.IMAGE_SCALE, im_height/self.IMAGE_SCALE), interpolation=cv2.INTER_LINEAR)
		
		# First check one of the frontal templates 
		if self.cascade_frontal_alt:
			faces = self.haar_detector(self.cascade_frontal_alt)
		
		# If a face is not found, try the profile template
		if len(faces) is 0:
			if self.cascade_profile:
				faces = self.haar_detector(self.cascade_profile)
			if len(faces) is 0:
				# If that fails, check a different frontal profile 
				if self.cascade_frontal_alt2:
					faces = self.haar_detector(self.cascade_frontal_alt2)

		# If we have found faces, generate a FaceBox message from them
		face_box = self.generate_face_box(faces)

		try:
			if face_box is None:
				print "noface"
				cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
				self.output_image_pub.publish(self.convert_cv_to_img(cv_image, encoding="bgr8"))
			else:
				print "face"
				self.output_face_box_pub.publish(face_box)
				self.face_detect_output_image_pub.publish(self.convert_cv_to_img(cv_image))

		except CvBridgeError, e:
			print e

	# Fuction to generate a list of face objects when given a classifier template
	def haar_detector(self, classifier):
		return classifier.detectMultiScale(self.small_image, self.HAAR_SCALE, self.MIN_NEIGHBORS, self.HAAR_FLAGS, self.MIN_SIZE)

	# Function to generate a FaceBox message type from a list of possible face objects
	def generate_face_box(self, faces):
		if len(faces) > 0:
			for (x, y, w, h) in faces:
				face_box = FaceBox()

				# The input to cv.HaarDetectObjects was resized, so scale the bounding box of each face and convert it to two CvPoints 
				pt1 = (int(x * self.IMAGE_SCALE), int(y * self.IMAGE_SCALE))
				pt2 = (int((x + w) * self.IMAGE_SCALE), int((y + h) * self.IMAGE_SCALE))
				face_width = pt2[0] - pt1[0]
				face_height = pt2[1] - pt1[1]

				face_box.x = pt1[0]
				face_box.y = pt1[1]
				face_box.width = face_width
				face_box.height = face_height

				return face_box
		return None


	# Abstract method implementations to allow scheduler commands to be processed
	def unsubscribe_node(self):
		# Function to unsubscribe a node from its topics and stop publishing data
		self.output_face_box_pub.unregister()
		self.output_image_pub.unregister()
		self.face_detect_output_image_pub.unregister()
		self.image_sub.unregister()

	def resubscribe_node(self):
		# Function to resubscribe and republish the nodes data
		self.output_face_box_pub = rospy.Publisher(self.face_box_coordinates, FaceBox, queue_size=self.queue_size)
		self.output_image_pub = rospy.Publisher(self.output_image, Image, queue_size=self.queue_size) 
		self.face_detect_output_image_pub = rospy.Publisher(self.face_detect_output_image, Image,queue_size=self.queue_size)
		self.image_sub = rospy.Subscriber(self.pre_processed_output_image, Image, self.detect_face, queue_size=self.queue_size)


def main(args):
	try:   
		# Fire up the node.
		FD = Face_Detector("face_detection_node")
		# Spin so our services will work
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down vision node."
		cv.DestroyAllWindows()

if __name__ == '__main__':

	main(sys.argv)
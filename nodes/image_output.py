#!/usr/bin/env python

import roslib
import rospy
import cv2
import sys
import threading
from offloadable_face_recognition.msg import FeatureCoordinates, CoordinatesList

from offloadable_fr_node import Offloadable_FR_Node
from sensor_msgs.msg import Image

class Post_Processing(Offloadable_FR_Node):

	def __init__(self, node_name):
		print "Initialising " + node_name

		Offloadable_FR_Node.__init__(self, node_name)

		self.image_sub = rospy.Subscriber(self.input_rgb_image, Image, self.post_processing, queue_size=self.queue_size)
		self.feature_coordinates = rospy.Subscriber(self.feature_coordinates_output, CoordinatesList, self.feature_coordinates_listener, queue_size=self.queue_size)
		self.output_image_pub = rospy.Publisher(self.output_image, Image, queue_size=self.queue_size)
		self.features = []
		self.features_lock = threading.Lock()

		self.COLOUR_FACE_BOX = (0,0,255) # BLUE
		self.COLOUR_FEATURE_POINTS = (0,255,0) # GREEN
		self.CV_FILLED = -1

	def feature_coordinates_listener(self, feature_coordinates):
		with self.features_lock:
			self.features = self.convert_to_tuple_array(feature_coordinates.coordinates)

	def post_processing(self, ros_image):
		cv_image = self.convert_img_to_cv(ros_image)
		im_width, im_height, im_channels = cv_image.shape
		post_processed_image = self.draw_graphics(cv_image, None ,self.features)
		post_processed_image = self.convert_cv_to_img(post_processed_image, encoding="rgb8")
		self.output_image_pub.publish(post_processed_image)

	def draw_graphics(self, cv_image, face_box, features):
		# take input image, check for whether there is a facebox or feature box
		# and if there is either, draw the appropriate graphics ontop of the
		# cv_image. Otherwise simply return the initial image. Returns feature matrix
		# Draw the points as green circles and add them to the features matrix 
		img = cv_image
		# If there is a face box then draw a rectangle around the region the face occupies
		with self.features_lock:
			if face_box and not (len(self.features) > 6):
				pt1 = (int(face_box.x), int(face_box.y))
				pt2 = (int(face_box.x+face_box.width), int(face_box.y+face_box.height))
				cv2.rectangle(img, pt1, pt2, self.COLOUR_FACE_BOX, thickness=2)

			# Otherwise if there are already features then draw the feature points as points on the face
			if len(features) > 6: #self.abs_min_features:
				for the_point in features:
					cv2.circle(img, (int(the_point[0]), int(the_point[1])), 2, self.COLOUR_FEATURE_POINTS,self.CV_FILLED)
			return img

	def unsubscribe_node(self):
		try:
			with self.offloading_lock:
				if self.is_offloaded == False:
					self.image_sub.unregister()
		except OffloadingError, e:
			print "Could not offload node " + self.node_name + "\n" + "-----\n" + e



	def resubscribe_node(self):
		with self.offloading_lock:
			# Function to resubscribe and republish the nodes data
			self.image_sub = rospy.Subscriber(self.input_rgb_image, Image, self.post_processing, queue_size=self.queue_size)
			self.feature_coordinates_subrospy.Subscriber(self.lk_tracker_coordinates, FeatureCoordinates, self.post_processing, queue_size=self.queue_size)

def main(args):
	try:   
		# Fire up the node.
		PP = Post_Processing("image_output_node")
		# Spin so our services will work
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down " + PP.node_name

if __name__ == '__main__':

	main(sys.argv)
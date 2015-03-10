#!/usr/bin/env python

import roslib
import rospy
import cv2
import sys
import numpy as np
import threading

from offloadable_fr_node import Offloadable_FR_Node
from cv_bridge import CvBridge, CvBridgeError
from offloadable_fr_node.errors import OffloadingError
from sensor_msgs.msg import Image
from offloadable_face_recognition.msg import FaceBox, SchedulerCommand, MotorCommand
from offloadable_face_recognition.srv import *

class LK_Tracker(Offloadable_FR_Node):

	def __init__(self, node_name):

		Offloadable_FR_Node.__init__(self, node_name)

		self.pyramid = None
		self.prev_pyramid = None
		self.grey = None
		self.prev_grey = None
		face_box = None
		self.features = []
		self.night_mode = False       
		self.WIN_SIZE = 10
		self.USE_HARRIS = False
		self.FLAGS = 0
		self.MAX_LEVEL = 2
		self.CV_FILLED = -1
		self.BAD_CLUSTER = False
		self.GOOD_CLUSTER = True
		self.COLOUR_FACE_BOX = (0,0,255) # BLUE
		self.COLOUR_FEATURE_POINTS = (0,255,0) # GREEN
		self.COLOUR_NO_FACE_TEXT = (255,0,0) # RED
		self.face_detected = False

		self.motor_commands = "motor_commands"

		self.camera_threshold_tolerance = 30 # %percent
		self.camera_x, self.camera_y = self.camera_dimensions
		self.camera_edge_threshold = self.camera_x/100*self.camera_threshold_tolerance

		self.abs_min_features = 6
		self.min_features = 5
		self.NO_IMAGE_TEXT = "NO FACE DETECTED!"
		self.face_box_lock = threading.Lock()
		self.subscriber_lock = threading.Lock()
		self.face_box = None
		self.feature_matrix = None
		self.marker_image = None
		self.pre_processed_image = None
		self.motor_commands_pub = None

		self.face_detected = True
		self.track_box

		# Parameters for lucas kande optical flow
		self.lk_params = dict( winSize  = (self.WIN_SIZE,self.WIN_SIZE),
						  maxLevel = self.MAX_LEVEL,
						  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		
		self.scheduler_sub = rospy.Subscriber(self.scheduler_commands, SchedulerCommand, self.scheduler_listener, queue_size=self.queue_size)
		self.output_image_pub = None
		self.image_sub = None
		self.face_box_sub = None
		self.motor_commands_pub = None

	def update_face_box(self, face_box):
		with self.face_box_lock:
			self.face_box = face_box
			self.face_detected = True

	def track_lk(self, ros_image):

		original_image = ros_image
		cv_image = self.convert_img_to_cv(ros_image)
		im_width, im_height = cv_image.shape

		with self.face_box_lock:
			if not self.track_box:
				self.features = []
				self.track_box = self.face_box

		# Switch between the incoming image streams depending on whether we have features or not
		if self.features!=[] and self.face_detected == False:
			with self.offloading_lock:
				self.face_box_sub.unregister()
				self.face_detected = True
		elif self.face_detected == True and (self.features==[] or self.track_box is None):
			with self.offloading_lock:
				self.face_box_sub = rospy.Subscriber(self.face_box_coordinates, FaceBox, self.update_face_box, queue_size=self.queue_size)
				self.face_detected = False

		#  Initialize intermediate images if necessary 
		if self.grey is None:
			self.grey = np.zeros((im_width,im_height,1), np.uint8)
			self.prev_grey = np.zeros((im_width,im_height,1), np.uint8)
			self.features = []
		
		self.marker_image = np.zeros((im_width,im_height,3), np.uint8)
		self.grey = cv_image

		if self.track_box and self.features != []:
			self.features, status, track_error = cv2.calcOpticalFlowPyrLK(self.prev_grey, self.grey, np.asarray(self.features,dtype="float32"), None, **self.lk_params)
			self.features = [ p for (st,p) in zip(status, self.features) if st]  #  Keep only high status points 
		elif self.track_box:
			self.features = self.add_features(ros_image, self.track_box, self.features)
			# Since the detect box is larger than the actual face or desired patch, shrink the number of features by 10% 

		# Swapping the images 
		
		# If we have some features... 
		if len(self.features) > 0:

			# The FitEllipse2 function below requires us to convert the feature array into a CvMat matrix 
			# Draw the best fit ellipse around the feature points 
			if len(self.features) > 6:
				self.feature_matrix = np.float32([p for p in self.features]).reshape(-1, 1, 2)  
				feature_box = cv2.fitEllipse(self.feature_matrix)
			else:
				feature_box = None

			if (len(self.features) < self.min_features) and (self.track_box is not None) and (feature_box is not None):
				self.expand_roi = self.expand_roi_init * self.expand_roi
				((self.track_box.x, self.track_box.y), (self.track_box.width, self.track_box.height), a) = feature_box
				self.track_box.width=self.track_box.width*self.expand_roi
				self.track_box.height=self.track_box.height*self.expand_roi
				self.features = self.add_features(ros_image, self.track_box, self.features)
			else:
				self.expand_roi = self.expand_roi_init

			self.features, score = self.prune_features(self.features, self.abs_min_features)

			if score == self.BAD_CLUSTER:
				self.features = []
				self.detect_box = None
				self.track_box = None

		self.prev_grey, self.grey = self.grey, self.prev_grey

		cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

		self.update_motor_position(self.features)

		cv_image = self.draw_graphics(cv_image, self.track_box, self.features)
		ros_image = self.convert_cv_to_img(cv_image, encoding="bgr8")

		with self.offloading_lock:
			if self.is_offloaded == False:
				try:
					if len(self.features) > 0:
						self.output_image_pub.publish(ros_image)
						"print tracking face"
					else:
						self.output_image_pub.publish(original_image)
				except OffloadingPublishError, e:
					print "Could publish data for" + self.node_name + "\n" + "-----\n" + e

		self.check_for_offload()

	def prune_features(self, prev_features, abs_min_features):
		# takes an array of feature coordinates and prunes them
		# returning the new set of features and a quality score.
		# Prune features that are too far from the main cluster
			rospy.wait_for_service('prune_features')
			prune_features = rospy.ServiceProxy('prune_features', PruneFeatures)

			try:
				response = prune_features(self.convert_to_feature_coordinates(prev_features), abs_min_features)

				features = self.convert_to_tuple_array(response.features)
				score = response.score

				return features, score

			except rospy.ServiceException as exc:
				self.features = []
				score = self.BAD_CLUSTER
				print("Service did not process request: " + str(exc))
					# Add features if the number is getting too low

			

	def add_features(self, ros_image, face_box, prev_features):
		# takes an image, a track box and an array of feature coordinates
		# and adds new features to this. Should return the array of 
		# new feature coordinates

		self.min_features = int(len(self.features) * 0.9)
		self.abs_min_features = int(0.5 * self.min_features)

		rospy.wait_for_service('add_features')
		add_features = rospy.ServiceProxy('add_features', AddFeatures)

		try:
			service_response = add_features(self.convert_to_feature_coordinates(prev_features), face_box, ros_image)
			features = self.convert_to_tuple_array(service_response.features)

			return features

		except rospy.ServiceException as exc:
			features = []
			print("Service did not process request: " + str(exc))

	def unsubscribe_node(self):
		# Function to unsubscribe a node from its topics and stop publishing data
		try:
			with self.offloading_lock:
				if self.is_offloaded == False:
					self.output_image_pub.unregister()
					self.motor_commands_pub.unregister()
					self.image_sub.unregister()
				
					if self.face_detected == False:
						self.face_box_sub.unregister()
						
		except OffloadingError, e:
			print "Could not offload node " + self.node_name + "\n" + "-----\n" + e

	def resubscribe_node(self):
		# Function to resubscribe and republish the nodes data
		with self.offloading_lock:
			self.output_image_pub = rospy.Publisher(self.output_image, Image, queue_size=self.queue_size)
			self.face_box_sub = rospy.Subscriber(self.face_box_coordinates, FaceBox, self.update_face_box, queue_size=self.queue_size)
			self.motor_commands_pub = rospy.Publisher(self.motor_commands, MotorCommand, queue_size=self.queue_size)
			self.image_sub = rospy.Subscriber(self.pre_processed_output_image, Image, self.track_lk, queue_size = self.queue_size)

def main(args):
	try:   
		# Fire up the node.
		LK = LK_Tracker("lk_tracker_node")
		# Spin so our services will work
		print "Node started..."
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down " + LK.node_name

if __name__ == '__main__':

	main(sys.argv)
#!/usr/bin/env python
import roslib
import rospy
import cv2
import sys
import numpy as np
import threading

from offloadable_fr_node import Offloadable_FR_Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from offloadable_face_recognition.msg import *
from offloadable_face_recognition.srv import *

class Prune_Features_Server(Offloadable_FR_Node):

	def __init__(self, node_name):
		Offloadable_FR_Node.__init__(self, node_name)
		
		self.BAD_CLUSTER = -1
		self.GOOD_CLUSTER = 1
		self.ABS_MIN_FEATURES = 6

		self.OUTLIER_THRESHOLD = 2.5
		self.MSE_THRESHOLD = 10000

		s = rospy.Service('prune_features', PruneFeatures, self.prune_features)

	def prune_features(self, request):
		prev_features = self.convert_to_tuple_array(request.prev_features)
		sum_x = 0
		sum_y = 0
		sse = 0
		features_len = len(prev_features)
		
		# Compute the COG (center of gravity) of the cluster 
		for point in prev_features:
			sum_x = sum_x + point[0]
			sum_y = sum_y + point[1]
		
		mean_x = sum_x / features_len
		mean_y = sum_y / features_len

		# Compute the x-y MSE (mean squared error) of the cluster in the camera plane 
		for point in prev_features:
			sse = sse + (point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)
			#sse = sse + abs((point[0] - mean_x)) + abs((point[1] - mean_y))
		
		# Get the average over the number of feature points 
		mse_xy = sse / features_len
		
		# The MSE must be > 0 for any sensible feature cluster 
		if mse_xy == 0 or mse_xy > self.MSE_THRESHOLD:
			response = self.convert_to_feature_coordinates(prev_features)
			return PruneFeaturesResponse(response, self.BAD_CLUSTER)
		
		# Throw away the outliers based on the x-y variance 
		max_err = 0
		for point in prev_features:
			std_err = ((point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)) / mse_xy
			if std_err > max_err:
				max_err = std_err
			if std_err > self.OUTLIER_THRESHOLD:
				prev_features.remove(point)
				
				features_len = features_len-1

		features = prev_features

		# Consider a cluster bad if we have fewer than abs_min_features left 
		if features_len < self.ABS_MIN_FEATURES:
			score = self.BAD_CLUSTER
		else:
			score = self.GOOD_CLUSTER

		response = self.convert_to_feature_coordinates(prev_features)
		
		return PruneFeaturesResponse(response, score)

	def unsubscribe_node(self):
		#unused
		return
		
	def resubscribe_node(self):
		#unused
		return

def main():
	try:
		PF = Prune_Features_Server("prune_features_server")
		print "Service awaiting requests..."
		rospy.spin()
	except rospy.ServiceException:
		print "Error adding new features"

if __name__ == "__main__":
	main()
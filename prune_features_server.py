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
from offloadable_face_recognition.msg import FaceBox
from offloadable_face_recognition.srv import PruneFeatures
def prune_features(request):

	sum_x = 0
	sum_y = 0
	sse = 0
	features = request.prev_features
	n_xy = len(prev_features)
	
	# If there are no features left to track, start over 
	if n_xy == 0:
		return ((0, 0, 0), 0, 0, -1)
	
	# Compute the COG (center of gravity) of the cluster 
	for point in prev_features:
		sum_x = sum_x + point[0]
		sum_y = sum_y + point[1]
	
	mean_x = sum_x / n_xy
	mean_y = sum_y / n_xy

	# Compute the x-y MSE (mean squared error) of the cluster in the camera plane 
	for point in prev_features:
		sse = sse + (point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)
		#sse = sse + abs((point[0] - mean_x)) + abs((point[1] - mean_y))
	
	# Get the average over the number of feature points 
	mse_xy = sse / n_xy
	
	# The MSE must be > 0 for any sensible feature cluster 
	if mse_xy == 0 or mse_xy > mse_threshold:
		return ((0, 0, 0), 0, 0, -1)
	
	# Throw away the outliers based on the x-y variance 
	max_err = 0
	for point in prev_features:
		std_err = ((point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)) / mse_xy
		if std_err > max_err:
			max_err = std_err
		if std_err > outlier_threshold:
			features.remove(point)
			
			n_xy = n_xy - 1

	# Consider a cluster bad if we have fewer than abs_min_features left 
	if len(features) < self.abs_min_features:
		score = -1
	else:
		score = 1

	return offloadable_face_recognition.srv.PruneFeaturesResponse(features, score)

def main():
    rospy.init_node('prune_features_server')
    s = rospy.Service('prune_features', PruneFeatures, prune_features)
    print "Service awaiting requests..."
    rospy.spin()

if __name__ == "__main__":
    main()
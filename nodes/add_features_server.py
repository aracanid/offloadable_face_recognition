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

class Add_Features_Server(Offloadable_FR_Node):

    def __init__(self, node_name):
        Offloadable_FR_Node.__init__(self, node_name)
        
        self.QUALITY = 0.01
        self.MAX_COUNT = 200
        self.BLOCK_SIZE = 3
        self.GOOD_FEATURE_DISTANCE = 5
        self.ADD_FEATURE_DISTANCE = 10

        self.CV_FILLED = -1

        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = self.MAX_COUNT,
                               qualityLevel = self.QUALITY,
                               minDistance = self.GOOD_FEATURE_DISTANCE,
                               blockSize = self.BLOCK_SIZE,
                               k = 0.04)

        s = rospy.Service('add_features', AddFeatures, self.add_features)

    def add_features(self, request):

        prev_features = self.convert_to_tuple_array(request.prev_features)

        track_box = request.track_box

        grey = self.convert_img_to_cv(request.ros_image)
        im_height, im_width = grey.shape

        #  Get the initial features to track 
                
        #  Create a mask image to be used to select the tracked points 
        mask = np.zeros((im_height,im_width,1), np.uint8)
        
        # Get the center of the track box (type CvRect) so we can create the equivalent CvBox2D (rotated rectangle) required by EllipseBox below. 
        center_x = int(track_box.x + track_box.width / 2)
        center_y = int(track_box.y + track_box.height / 2)
        roi_box = ((center_x, center_y), (track_box.height, track_box.width), 0)
        
        # Create a filled white ellipse within the track_box to define the ROI. 
        cv2.ellipse(mask, roi_box, (255,255, 255), self.CV_FILLED)  
        
        new_features = cv2.goodFeaturesToTrack(grey, mask=mask, **self.feature_params)

        features = prev_features

        if new_features is not None:
            for x, y in np.float32(new_features).reshape(-1, 2):
                distance = self.distance_to_cluster((x,y), features)
                if distance > self.ADD_FEATURE_DISTANCE:
                    features.append((x,y))

        # Remove duplicate features 
        features = list(set(features))
    
        response = self.convert_to_feature_coordinates(features)
        
        return AddFeaturesResponse(response)

    def distance_to_cluster(self, test_point, cluster):
        min_distance = 10000
        for point in cluster:
            if point == test_point:
                continue
            # Use L1 distance since it is faster than L2
            distance = abs(test_point[0] - point[0]) + abs(test_point[1] - point[1])
            if distance < min_distance:
                min_distance = distance
        return min_distance

def main():
    try:
        AF = Add_Features_Server("add_features_server")
        print "Service awaiting requests..."
        rospy.spin()
    except rospy.ServiceException:
        print "Error adding new features"

if __name__ == "__main__":
    main()
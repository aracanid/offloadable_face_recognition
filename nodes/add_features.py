import roslib
import rospy
import cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import PointStamped
from offloadable_face_recognition.msg import FaceBox
from cv_bridge import CvBridge, CvBridgeError
from offloadable_fr_node import Offloadable_FR_Node


def add_features(request):

    prev_features = request.prev_features
    track_box = request.track_box

    grey = convert_img_to_cv(request.ros_image)

    # Look for any new features around the current feature cloud 
    
    # Create the ROI mask
    roi = cv.CreateImage(image_size, 8, 1) 
    
    # Begin with all black pixels 
    cv.Zero(roi)
    
    # Get the coordinates and dimensions of the current track box 
    try:
        x = track_box.point0
        y = track_box.point1
        w = track_box.width
        h = track_box.height
    except:
        rospy.loginfo("Track box has shrunk to zero...")
        return
    
    # Expand the track box to look for new features 
    w = int(self.expand_roi * w)
    h = int(self.expand_roi * h)
    
    roi_box = ((x,y), (w,h), a)
    
    # Create a filled white ellipse within the track_box to define the ROI. 
    cv.EllipseBox(roi, roi_box, cv.CV_RGB(255,255, 255), cv.CV_FILLED)
    
    # Create the temporary scratchpad images 
    eig = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
    temp = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
    
    # Get the new features using Good Features to Track 
    features = cv.GoodFeaturesToTrack(self.grey, eig, temp, self.max_count,
    self.quality, self.good_feature_distance, mask=roi, blockSize=3, useHarris=0, k=0.04)
    
    # Append new features to the current list if they are not too far from the current cluster 
    for new_feature in features:
        try:
            distance = self.distance_to_cluster(new_feature, prev_features)
            if distance > self.add_feature_distance:
                prev_features.append(new_feature)
        except:
            pass
            
    # Remove duplicate features 
    response = list(set(self.features))

    return offloadable_face_recognition.srv.AddFeaturesResponse(response)

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
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
        self.QUALITY = 0.01
        self.WIN_SIZE = 10
        self.MAX_COUNT = 200
        self.BLOCK_SIZE = 3
        self.USE_HARRIS = False
        self.FLAGS = 0
        self.MAX_LEVEL = 2
        self.GOOD_FEATURE_DISTANCE = 5

        self.NO_IMAGE_TEXT = "NO FACE DETECTED!"
        self.lock = threading.Lock()
        self.face_box = None

        # params for ShiTomasi corner detection
        feature_params = dict( maxCorners = self.MAX_COUNT,
                               qualityLevel = self.QUALITY,
                               minDistance = self.GOOD_FEATURE_DISTANCE,
                               blockSize = self.BLOCK_SIZE,
                               useHarris = self.USE_HARRIS,
                               k = 0.04)

        # Parameters for lucas kanade optical flow
        lk_params = dict( winSize  = (self.WIN_SIZE,self.WIN_SIZE),
                          maxLevel = self.MAX_LEVEL,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


        # A publisher to output the display image back to a ROS topic 
        self.output_image_pub = rospy.Publisher(self.output_image, Image, queue_size=self.queue_size)

        # Subscribe to the raw camera image topic and set the image processing callback 
        self.image_sub = rospy.Subscriber(self.face_detect_output_image, Image, self.track_lk, queue_size=self.queue_size)
        self.face_box_sub = rospy.Subscriber(self.face_box_coordinates, FaceBox, self.update_face_box, queue_size=self.queue_size)
        # Subscribe to the preprocessed image output and set the detect_face as the callback
        #image_sub = rospy.Subscriber(self.marker_image_output, Image, self.update_marker_image, queue_size=self.queue_size)

    def update_face_box(self, face_box):
        with self.lock:
            self.face_box = face_box

    def track_lk(self, ros_image):

        cv_image = self.convert_img_to_cv(ros_image)
        im_width, im_height = cv_image.shape

        if self.face_box_sub is None:
            print "before lock"
            with self.lock:
                face_box = self.face_box
            print "after lock"
        else:
            face_box = None

        print "before subscriber changing"
        # Switch between the incoming image streams depending on whether we have features or not
        if len(self.features) > 0:
            self.image_sub = rospy.Subscriber(self.pre_processed_output_image, Image, self.track_lk, self.queue_size)
            self.face_box_sub = None
            print "switched topic to tracking only"

        elif self.face_box_sub is None and len(self.features) is 0:
            self.image_sub = rospy.Subscriber(self.face_detect_output_image, Image, self.track_lk, self.queue_size)
            self.face_box_sub = rospy.Subscriber(self.face_box_coordinates, FaceBox, self.track_lk, self.queue_size)

            text_scale = 0.4 * im_width / 160. + 0.1
            text_pos = (100, im_height-100)
            cv2.putText(self.marker_image, self.NO_IMAGE_TEXT, text_pos, cv2.FONT_HERSHEY_COMPLEX, text_scale, (255,0,0), thickness=4)
            print "switched topic back to face detector"
        feature_box = None
        
        #  Initialize intermediate images if necessary 
        if not self.pyramid:
            self.grey = np.zeros((im_width,im_height,1), np.uint8)
            self.prev_grey = np.zeros((im_width,im_height,1), np.uint8)
            self.features = []
        

        self.grey = cv_image

        print "before opt flow"
            
        if face_box and self.features != []:
            #  We have feature points, so track and display them 

            #  Calculate the optical flow 

            self.features, status, track_error = cv2.calcOpticalFlowPyrLK(self.prev_grey, self.grey, self.features, None, **lk_params)

            #  Keep only high status points 
            self.features = [ p for (st,p) in zip(status, self.features) if st]  
        

        elif face_box and self.is_rect_nonzero(face_box):
            print "before good features"
            #  Get the initial features to track 
                    
            #  Create a mask image to be used to select the tracked points 
            mask = np.zeros((im_width,im_height,1), np.uint8)
            
            # Get the center of the track box (type CvRect) so we can create the equivalent CvBox2D (rotated rectangle) required by EllipseBox below. 
            center_x = int(face_box.x + face_box.width / 2)
            center_y = int(face_box.y + face_box.height / 2)
            roi_box = ((center_x, center_y), (face_box.width, face_box.height), 0)
            
            # Create a filled white ellipse within the track_box to define the ROI. 
            cv.EllipseBox(mask, roi_box, cv.CV_RGB(255,255, 255), cv.CV_FILLED)      
            
            # Create the temporary scratchpad images 
            eig = np.zeros((im_width,im_height,1), np.uint32)
            temp = np.zeros((im_width,im_height,1), np.uint32)

            self.features = cv2.goodFeaturesToTrack(self.grey, mask=mask, **feature_params)
            
            # Since the detect box is larger than the actual face or desired patch, shrink the number of features by 10% 
            self.min_features = int(len(self.features) * 0.9)
            self.abs_min_features = int(0.5 * self.min_features)
        
        # Swapping the images 
        self.prev_grey, self.grey = self.grey, self.prev_grey
        

        # If we have some features... 
        if len(self.features) > 0:
            print "we have features"
            # The FitEllipse2 function below requires us to convert the feature array into a CvMat matrix 
            try:
                self.feature_matrix = np.zeros((1,len(self.features,1),cv2.CV_32SC2))
            except:
                pass
                        
            # Draw the points as green circles and add them to the features matrix 
            i = 0
            for the_point in self.features:
                if self.show_features:
                    cv2.circle(self.marker_image, (int(the_point[0]), int(the_point[1])), 2, (0, 255, 0, 0), cv.CV_FILLED, 8, 0)
                try:
                    self.feature_matrix[0][i] = (int(the_point[0]), int(the_point[1]))
                except:
                    pass
                i = i + 1
    
            # Draw the best fit ellipse around the feature points 
            if len(self.features) > 6:
                feature_box = cv2.fitEllipse(self.feature_matrix)
            else:
                feature_box = None

            # Prune features that are too far from the main cluster
            rospy.wait_for_service('prune_features')
            prune_features = rospy.ServiceProxy('prune_features', PruneFeatures)

            try:
                (self.features, score) = PruneFeatures(self.features)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            
            if score == -1:
                self.detect_box = None
                face_box = None
            

            self.pre_processed_image = ns.concatenate(self.marker_image,cv2.cvtColor(self.pre_processed_image, cv2.COLOR_GRAY2BGR))
            ros_image = convert_cv_to_img(self.pre_processed_image)

            # Add features if the number is getting too low
            if len(self.features) < self.min_features:
                self.expand_roi = self.expand_roi_init * self.expand_roi

                ((face_box.x, face_box.y), (face_box.width, face_box.height), a) = self.feature_box
                
                rospy.wait_for_service('add_features')
                prune_features = rospy.ServiceProxy('add_features', AddFeatures)

                try:
                    self.features = AddFeatures(self.features, face_box, ros_image)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
            else:
                self.expand_roi = self.expand_roi_init    

        if feature_box is not None and len(self.features) > 0:
            try:
                self.output_image_pub.publish(ros_image)
            except CvBridgeError, e:
                print e

def main(args):
    try:   
        # Fire up the node.
        LK = LK_Tracker("ev3_lk_tracker")
        # Spin so our services will work
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':

    main(sys.argv)
import roslib
import rospy
import cv
import sys
from sensor_msgs.msg import RegionOfInterest, Image
from math import sqrt, isnan
from ros2opencv import ROS2OpenCV
from pi_face_tracker.srv import *
from offloadable_fr_node import Offloadable_FR_Node
from offloadable_face_recognition.msg import FaceBox
from offloadable_face_recognition.msg import FaceBox

class EV3_LK_Tracker(Offloadable_FR_Node):

    def __init__(self, node_name):

        self.pyramid = None
        self.prev_pyramid = None
        self.grey = None
        self.prev_grey = None
        self.track_box = None
        self.features = []
        self.night_mode = False       
        self.quality = 0.01
        self.win_size = 10
        self.max_count = 200
        self.block_size = 3
        self.use_harris = False
        self.flags = 0
        self.frame_count = 0

        # A publisher to output the display image back to a ROS topic 
        self.output_image_pub = rospy.Publisher("output_image", Image)

        # Subscribe to the raw camera image topic and set the image processing callback 
        self.image_sub = rospy.Subscriber("face_detect_output_image", Image, self.track_lk, self.queue_size)
        self.face_box_sub = rospy.Subscriber("face_box_coordinates", FaceBox, self.track_lk, self.queue_size)


        self.face_box = FaceBox()


    def update_face_box(self, face_box):
        self.track_box = (face_box.point0, face_box.point1, face_box.width, face_box.height)

    def track_lk(self, ros_image):

        cv_image = convert_img_to_cv(ros_image)

        # Switch between the incoming image streams depending on whether we have features or not
        if len(self.features) > 0:
            self.image_sub = rospy.Subscriber("pre_processed_image", Image, self.track_lk, self.queue_size)
            self.face_box_sub = None
        elif self.face_box_sub is None and len(self.features) is 0:
            self.image_sub = rospy.Subscriber("face_detect_output_image", Image, self.track_lk, self.queue_size)
            self.face_box_sub = rospy.Subscriber("face_box_coordinates", FaceBox, self.track_lk, self.queue_size)

        feature_box = None
        
        #  Initialize intermediate images if necessary 
        if not self.pyramid:
            self.grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.features = []
        

        self.grey = cv_image
            
        if self.track_box and self.features != []:
            #  We have feature points, so track and display them 

            #  Calculate the optical flow 
            self.features, status, track_error = cv.CalcOpticalFlowPyrLK(
                self.prev_grey, self.grey, self.prev_pyramid, self.pyramid,
                self.features,
                (self.win_size, self.win_size), 3,
                (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.01),
                self.flags)

            #  Keep only high status points 
            self.features = [ p for (st,p) in zip(status, self.features) if st]        
                                    
        elif self.track_box and self.is_rect_nonzero(self.track_box):
            #  Get the initial features to track 
                    
            #  Create a mask image to be used to select the tracked points 
            mask = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
            
            #  Begin with all black pixels 
            cv.Zero(mask)

            #  Get the coordinates and dimensions of the track box 
            try:
                x,y,w,h = self.track_box
            except:
                return None
            
            if self.auto_face_tracking:
#                 For faces, the detect box tends to extend beyond the actual object so shrink it slightly 
#                x = int(0.97 * x)
#                y = int(0.97 * y)
#                w = int(1 * w)
#                h = int(1 * h)
                
                # Get the center of the track box (type CvRect) so we can create the equivalent CvBox2D (rotated rectangle) required by EllipseBox below. 
                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                roi_box = ((center_x, center_y), (w, h), 0)
                
                # Create a filled white ellipse within the track_box to define the ROI. 
                cv.EllipseBox(mask, roi_box, cv.CV_RGB(255,255, 255), cv.CV_FILLED)      
            else:
                # For manually selected regions, just use a rectangle 
                pt1 = (x, y)
                pt2 = (x + w, y + h)
                cv.Rectangle(mask, pt1, pt2, cv.CV_RGB(255,255, 255), cv.CV_FILLED)
            
            # Create the temporary scratchpad images 
            eig = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
            temp = cv.CreateImage (cv.GetSize(self.grey), 32, 1)

            if self.feature_type == 0:
                # Find keypoints to track using Good Features to Track 
                self.features = cv.GoodFeaturesToTrack(self.grey, eig, temp, self.max_count,
                    self.quality, self.good_feature_distance, mask=mask, blockSize=self.block_size, useHarris=self.use_harris, k=0.04)
            
            elif self.feature_type == 1:
                # Get the new features using SURF 
                (surf_features, descriptors) = cv.ExtractSURF(self.grey, mask, cv.CreateMemStorage(0), (0, self.surf_hessian_quality, 3, 1))
                for feature in surf_features:
                    self.features.append(feature[0])
            
            if self.auto_min_features:
                # Since the detect box is larger than the actual face or desired patch, shrink the number of features by 10% 
                self.min_features = int(len(self.features) * 0.9)
                self.abs_min_features = int(0.5 * self.min_features)
        
        # Swapping the images 
        self.prev_grey, self.grey = self.grey, self.prev_grey
        self.prev_pyramid, self.pyramid = self.pyramid, self.prev_pyramid
        
        # If we have some features... 
        if len(self.features) > 0:
            # The FitEllipse2 function below requires us to convert the feature array into a CvMat matrix 
            try:
                self.feature_matrix = cv.CreateMat(1, len(self.features), cv.CV_32SC2)
            except:
                pass
                        
            # Draw the points as green circles and add them to the features matrix 
            i = 0
            for the_point in self.features:
                if self.show_features:
                    cv.Circle(self.marker_image, (int(the_point[0]), int(the_point[1])), 2, (0, 255, 0, 0), cv.CV_FILLED, 8, 0)
                try:
                    cv.Set2D(self.feature_matrix, 0, i, (int(the_point[0]), int(the_point[1])))
                except:
                    pass
                i = i + 1
    
            # Draw the best fit ellipse around the feature points 
            if len(self.features) > 6:
                feature_box = cv.FitEllipse2(self.feature_matrix)
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
                self.track_box = None
            
            ros_image = convert_cv_to_img(self.pre_processed_image)

            # Add features if the number is getting too low
            if len(self.features) < self.min_features:
                self.expand_roi = self.expand_roi_init * self.expand_roi

                ((self.face_box.point0, self.face_box.point1), (self.face_box.width, self.face_box.height), a) = self.feature_box
                
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
            LK = EV3_LK_Tracker("ev3_lk_tracker")
            # Spin so our services will work
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down vision node."
            cv.DestroyAllWindows()

    if __name__ == '__main__':
        # Create the cv_bridge object 

        rospy.loginfo("Starting " + node_name)

        main(sys.argv)
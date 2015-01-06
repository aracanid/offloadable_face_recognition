import roslib
import rospy
import cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import time

class Offloadable_FR_Node:
	def __init__(self, node_name):
		rospy.init_node(node_name)
		
		rospy.on_shutdown(self.cleanup)
		
		self.node_name = node_name
		self.input_rgb_image = "input_rgb_image"
		self.input_depth_image = "input_depth_image"
		self.output_image = "output_image"
		self.show_text = rospy.get_param("~show_text", True)
		self.show_features = rospy.get_param("~show_features", True)

		# Initialize the Region of Interest and its publisher 
		self.ROI = RegionOfInterest()
		self.pubROI = rospy.Publisher("/roi", RegionOfInterest)

		# Do the same for the point cluster publisher 
		self.cluster3d = PointStamped()
		self.pub_cluster3d = rospy.Publisher("/target_point", PointStamped)
		
		# Initialize a number of global variables 
		self.image = None
		self.image_size = None
		self.depth_image = None
		self.grey = None
		self.selected_point = None
		self.selection = None
		self.drag_start = None
		self.keystroke = None
		self.key_command = None
		self.detect_box = None
		self.track_box = None
		self.display_box = None
		self.keep_marker_history = False
		self.night_mode = False
		self.auto_face_tracking = True
		self.cps = 0 # Cycles per second = number of processing loops per second.
		self.cps_values = list()
		self.cps_n_values = 20
		self.flip_image = False

		# Create the display window 
		self.cv_window_name = self.node_name
		cv.NamedWindow(self.cv_window_name, cv.CV_NORMAL)
		cv.ResizeWindow(self.cv_window_name, 640, 480)
		
		# Create the cv_bridge object 
		self.bridge = CvBridge()

		self.auto_face_tracking = rospy.get_param("~auto_face_tracking", True)
		self.use_haar_only = rospy.get_param("~use_haar_only", False)
		self.use_depth_for_detection = rospy.get_param("~use_depth_for_detection", False)
		self.fov_width = rospy.get_param("~fov_width", 1.094)
		self.fov_height = rospy.get_param("~fov_height", 1.094)
		self.max_face_size = rospy.get_param("~max_face_size", 0.28)
		self.use_depth_for_tracking = rospy.get_param("~use_depth_for_tracking", False)
		self.auto_min_features = rospy.get_param("~auto_min_features", True)
		self.min_features = rospy.get_param("~min_features", 50) # Used only if auto_min_features is False
		self.abs_min_features = rospy.get_param("~abs_min_features", 6)
		self.std_err_xy = rospy.get_param("~std_err_xy", 2.5)
		self.pct_err_z = rospy.get_param("~pct_err_z", 0.42) 
		self.max_mse = rospy.get_param("~max_mse", 10000)
		self.good_feature_distance = rospy.get_param("~good_feature_distance", 5)
		self.add_feature_distance = rospy.get_param("~add_feature_distance", 10)
		self.flip_image = rospy.get_param("~flip_image", False)
		self.feature_type = rospy.get_param("~feature_type", 0) # 0 = Good Features to Track, 1 = SURF
		self.expand_roi_init = rospy.get_param("~expand_roi", 1.02)
		self.expand_roi = self.expand_roi_init
		
		self.camera_frame_id = "kinect_depth_optical_frame"
		
		self.cog_x = self.cog_y = 0
		self.cog_z = -1
			
		self.detect_box = None
		self.track_box = None
		self.features = []
		
		self.grey = None
		self.pyramid = None
		self.small_image = None
		
		# Set up the face detection parameters 
		self.cascade_frontal_alt = rospy.get_param("~cascade_frontal_alt", "")
		self.cascade_frontal_alt2 = rospy.get_param("~cascade_frontal_alt2", "")
		self.cascade_profile = rospy.get_param("~cascade_profile", "")
		
		self.cascade_frontal_alt = cv.Load(self.cascade_frontal_alt)
		self.cascade_frontal_alt2 = cv.Load(self.cascade_frontal_alt2)
		self.cascade_profile = cv.Load(self.cascade_profile)

		self.min_size = (20, 20)
		self.image_scale = 2
		self.haar_scale = 1.5
		self.min_neighbors = 1
		self.haar_flags = cv.CV_HAAR_DO_CANNY_PRUNING
		
		self.grey = None
		self.pyramid = None
		
		# Set the Good Features to Track and Lucas-Kanade parameters 
		self.night_mode = False       
		self.quality = 0.01
		self.win_size = 10
		self.max_count = 200
		self.block_size = 3
		self.use_harris = False
		self.flags = 0
		
		self.frame_count = 0
		
		# Set the SURF parameters 
		self.surf_hessian_quality = rospy.get_param("~surf_hessian_quality", 100)

		self.queue_size = 1

	def convert_img_to_cv(self, ros_image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
			return cv_image
		except CvBridgeError, e:
		  print e

	def convert_cv_to_img(self, cv_image):
		try:
			cv_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
			return cv_image
		except CvBridgeError, e:
		  print e
		  
	def is_rect_nonzero(self, r):
		# First assume a simple CvRect type
		try:
			(_,_,w,h) = r
			return (w > 0) and (h > 0)
		except:
			try:
				# Otherwise, assume a CvBox2D type
				((_,_),(w,h),a) = r
				return (w > 0) and (h > 0)
			except:
				return False
		
	def cleanup(self):
		print "Shutting down vision node."
		cv.DestroyAllWindows()  


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


class EV3_Face_Detector(Offloadable_FR_Node):

	def __init__(self, node_name):

		self.cv_image = None
		self.small_image = None
		self.min_size = (20, 20)
		self.image_scale = 2
		self.haar_scale = 1.5
		self.min_neighbors = 1
		self.haar_flags = cv.CV_HAAR_DO_CANNY_PRUNING

		self.cascade_frontal_alt = rospy.get_param("~cascade_frontal_alt", "")
		self.cascade_frontal_alt2 = rospy.get_param("~cascade_frontal_alt2", "")
		self.cascade_profile = rospy.get_param("~cascade_profile", "")
		
		self.cascade_frontal_alt = cv.Load(self.cascade_frontal_alt)
		self.cascade_frontal_alt2 = cv.Load(self.cascade_frontal_alt2)
		self.cascade_profile = cv.Load(self.cascade_profile)

		self.face_box = FaceBox()

    	# A publisher to output the display image back to a ROS topic 
		output_face_box_pub = rospy.Publisher("face_box_coordinates", FaceBox)  ###need to change this so that it sends an array

		# A publisher to output the display image back to a ROS topic 
		output_image_pub = rospy.Publisher("face_detect_output_image", Image)  ###need to change this so that it sends an array

		# Subscribe to the raw camera image topic and set the image processing callback 
		image_sub = rospy.Subscriber("pre_processed_image", Image, detect_face, self.queue_size)

	def detect_face(ros_image):

		self.cv_image = convert_img_to_cv(ros_image)

		if self.small_image is None:
            self.small_image = cv.CreateImage((cv.Round(self.image_size[0] / self.image_scale),
                       cv.Round(self.image_size[1] / self.image_scale)), 8, 1)

		# Scale input image for faster processing 
		cv.Resize(cv_image, self.small_image, cv.CV_INTER_LINEAR)
	
		# First check one of the frontal templates 
		if self.cascade_frontal_alt:
			faces = cv.HaarDetectObjects(self.small_image, self.cascade_frontal_alt, cv.CreateMemStorage(0),
										  self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
										 
		# If that fails, check the profile template 
		if not faces:
			if self.cascade_profile:
				faces = cv.HaarDetectObjects(self.small_image, self.cascade_profile, cv.CreateMemStorage(0),
											 self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)

			if not faces:
				# If that fails, check a different frontal profile 
				if self.cascade_frontal_alt2:
					faces = cv.HaarDetectObjects(self.small_image, self.cascade_frontal_alt2, cv.CreateMemStorage(0),
										 self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
			
		if not faces:
			hscale = 0.4 * self.image_size[0] / 160. + 0.1
			vscale = 0.4 * self.image_size[1] / 120. + 0.1
			text_font = cv.InitFont(cv.CV_FONT_VECTOR0, hscale, vscale, 0, 1, 8)
			cv.PutText(self.marker_image, "NO FACE DETECTED!", (50, int(self.image_size[1] * 0.9)), text_font, cv.RGB(255, 255, 0))
			
			return None
				
		for ((x, y, w, h), n) in faces:
			# The input to cv.HaarDetectObjects was resized, so scale the bounding box of each face and convert it to two CvPoints 
			pt1 = (int(x * self.image_scale), int(y * self.image_scale))
			pt2 = (int((x + w) * self.image_scale), int((y + h) * self.image_scale))
			face_width = pt2[0] - pt1[0]
			face_height = pt2[1] - pt1[1]

			self.face_box.point0 = pt1[0]
			self.face_box.point1 = pt1[1]
			self.face_box.width = face_width
			self.face_box.height = face_height

			# Break out of the loop after the first face 
			try:
				self.output_image_pub.publish(convert_cv_to_img(self.pre_processed_image))
				self.output_face_box_pub.publish(self.face_box)
			except CvBridgeError, e:
				print e

	def main(args):
		try:   
			# Fire up the node.
			FD = EV3_Face_Detector("ev3_face_detector")
			# Spin so our services will work
			rospy.spin()
		except KeyboardInterrupt:
			print "Shutting down vision node."
			cv.DestroyAllWindows()

	if __name__ == '__main__':
		# Create the cv_bridge object 

		rospy.loginfo("Starting " + node_name)

		main(sys.argv)
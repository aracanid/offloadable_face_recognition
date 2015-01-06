import roslib
import rospy
import cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from offloadable_fr_node import Offloadable_FR_Node
import time



class EV3_Image_Pre_Processing(Offloadable_FR_Node):

	def __init__(self, node_name):

        self.grey = None
        self.cv_image = None
        self.image_size = None
        self.image = None
        self.pre_processed_image = None
        
    	# A publisher to output the processed image back to a ROS topic
		pre_processed_image_pub = rospy.Publisher("pre_processed_image", Image) ###change

		# Wait until the image topics are ready before starting
        rospy.wait_for_message("input_rgb_image", Image)

		# Subscribe to the raw camera image topic and set the image processing callback
		image_sub = rospy.Subscriber("input_rgb_image", Image, detect_face, self.queue_size)

	def pre_processing(ros_image):
		
		# Convert the raw image to OpenCV format using the convert_img_to_cv() helper function
		self.cv_image = convert_img_to_cv(ros_image)
		
		# Create a few images we will use for display
		if not self.image:
			self.image_size = cv.GetSize(cv_image)
			self.image = cv.CreateImage(self.image_size, 8, 3)
			self.pre_processed_image = cv.CreateImage(self.image_size, 8, 3)

		if self.grey is None:
			# Allocate temporary images     
			self.grey = cv.CreateImage(self.image_size, 8, 1)

		# Convert color input image to grayscale 
		cv.CvtColor(self.cv_image, self.grey, cv.CV_BGR2GRAY)

		# Equalize the histogram to reduce lighting effects. 
		cv.EqualizeHist(self.grey, self.grey)
		
		self.pre_processed_image = self.grey
		
		# Publish the display image back to ROS 
		try:
			self.pre_processed_image_pub.publish(convert_cv_to_img(self.pre_processed_image))
		except CvBridgeError, e:
			print e

	def main(args):
		try:   
			# Fire up the node.
			PP = EV3_Image_Pre_Processing("ev3_image_pre_processing")
			# Spin so our services will work
			rospy.spin()
		except KeyboardInterrupt:
			print "Shutting down vision node."
			cv.DestroyAllWindows()

	if __name__ == '__main__':
		rospy.loginfo("Starting " + node_name)

		main(sys.argv)
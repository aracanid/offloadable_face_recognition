#!/usr/bin/env python

import roslib
import rospy
import cv

class Post_Processing(Offloadable_FR_Node):

	def __init__(self, node_name):
		print "Initialising " + node_name

		Offloadable_FR_Node.__init__(self, node_name)

		self.image_sub = None



	def post_processing(self, feature_coordinates):


	def draw_graphics(self, cv_image, face_box, features):
		# take input image, check for whether there is a facebox or feature box
		# and if there is either, draw the appropriate graphics ontop of the
		# cv_image. Otherwise simply return the initial image. Returns feature matrix
		# Draw the points as green circles and add them to the features matrix 

		# If there is a face box then draw a rectange around the region the face occupies
		if face_box and len(self.features) > self.abs_min_features:
			pt1 = (int(face_box.x), int(face_box.y))
			pt2 = (int(face_box.x+face_box.width), int(face_box.y+face_box.height))
			cv2.rectangle(cv_image, pt1, pt2, self.COLOUR_FACE_BOX, thickness=2)

		# Otherwise if there are already features then draw the feature points as points on the face
		if len(features) > self.abs_min_features:
			for the_point in features:
				cv2.circle(cv_image, (int(the_point[0]), int(the_point[1])), 2, self.COLOUR_FEATURE_POINTS,self.CV_FILLED)

		return cv_image

	def unsubscribe_node(self):
		try:
			with self.offloading_lock:
				if self.is_offloaded == False:
					self.image_sub.unregister()
		except OffloadingError, e:
			print "Could not offload node " + self.node_name + "\n" + "-----\n" + e



	def resubscribe_node(self):
		with self.offloading_lock:
			# Function to resubscribe and republish the nodes data
			self.image_sub = rospy.Subscriber(self.input_rgb_image, Image, self.post_processing, queue_size=self.queue_size)
			self.feature_coordinates_subrospy.Subscriber(self.lk_tracker_coordinates, FeatureCoordinates, self.post_processing, queue_size=self.queue_size)

def main(args):
	try:   
		# Fire up the node.
		PP = Post_Processing("image_output_node")
		# Spin so our services will work
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down " + PP.node_name

if __name__ == '__main__':

	main(sys.argv)
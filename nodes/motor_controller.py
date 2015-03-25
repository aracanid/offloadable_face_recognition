#!/usr/bin/env python

import socket
import sys
import rospy 
from offloadable_face_recognition.msg import MotorCommand, FeatureCoordinates, CoordinatesList
from OffloadingError import OffloadingError, OffloadingPublishError
from sensor_msgs.msg import Image
import time
import struct

class Motor_Controller:

	def __init__(self):

		node_name = "motor_controller"

		rospy.init_node(node_name)

		self.PORT_NUMBER = 5004
		self.DATA_SIZE = 1024
		self.socket = None
		self.MOTOR_COMMANDS = "motor_commands"
		self.feature_coordinates_output = "feature_coordinates"
		self.input_rgb_image = "input_rgb_image"
		self.queue_size = 1
		self.packer = struct.Struct('f f')

		self.YAW_LEFT = "yaw_left "
		self.YAW_RIGHT = "yaw_right"

		#motor_server_ip = raw_input("Please enter the IP address of the motor_server: ")
		motor_server_ip = "10.42.0.3"
		
		print "Initialising connection..."
		self.initialise_connection(motor_server_ip)
		self.feature_coordinates = rospy.Subscriber(self.feature_coordinates_output, CoordinatesList, self.feature_coordinates_listener, queue_size=self.queue_size)

	def feature_coordinates_listener(self, feature_coordinates):
		feature_coordinates = self.convert_to_tuple_array(feature_coordinates.coordinates)
		x, y = self.get_features_center(feature_coordinates)
		values = (x, y)
		packed_data = self.packer.pack(*values)
		self.socket.sendall(packed_data)
	
	def initialise_connection(self, ip_addr):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.connect((ip_addr, self.PORT_NUMBER))


	#Should move this into the motor controller and simply have
	#the motor controller recieve the current position?
	def get_features_center(self, features):
		# take the center of the current ellipse as the mean point
		# and sends the data to the motor controller
		features_len = len(features)
		# Compute the COG (center of gravity) of the cluster 
		if features_len > 0:
			sum_x, sum_y = 0, 0
			for point in features:
				sum_x = sum_x + point[0]
				sum_y = sum_y + point[1]
			
			center_x = sum_x / features_len
			center_y = sum_y / features_len

			return center_x, center_y


	def convert_to_tuple_array(self, feature_coordinates):
		converted_array = []
		
		fc_array = list(feature_coordinates)

		for fc in fc_array:
			tuple_coordinates = tuple(fc.coordinates)
			converted_array.append(tuple_coordinates)

		return converted_array

def main(args):
	try:   
		# Fire up the node.
		MC = Motor_Controller()
		# Spin so our services will work
		print "Node started..."
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down " + MC.node_name

if __name__ == '__main__':
	main(sys.argv)
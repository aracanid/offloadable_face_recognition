#!/usr/bin/env python

import socket
import sys
import rospy 
from offloadable_face_recognition.msg import MotorCommand
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
		self.queue_size = 1
		self.packer = struct.Struct('9s I')

		self.YAW_LEFT = "yaw_left "
		self.YAW_RIGHT = "yaw_right"

		#motor_server_ip = raw_input("Please enter the IP address of the motor_server: ")
		motor_server_ip = "10.42.0.3"
		
		print "Initialising connection..."
		self.initialise_connection(motor_server_ip)

		self.motor_commands = rospy.Subscriber(self.MOTOR_COMMANDS, MotorCommand, self.motor_command_listener, queue_size=self.queue_size)

	def motor_command_listener(self, motor_command):
		self.send_command(motor_command.command, motor_command.angle)
	
	def initialise_connection(self, ip_addr):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.connect((ip_addr, self.PORT_NUMBER))

	def send_command(self, command, angle):
		values = (command, angle)
		packed_data = self.packer.pack(*values)
		self.socket.sendall(packed_data)


def main(args):
	try:   
		# Fire up the node.
		MC = Motor_Controller()
		# Spin so our services will work
		print "Node started..."
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down vision node."
		cv.DestroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
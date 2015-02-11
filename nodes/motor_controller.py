#!/usr/bin/env python

import socket
import sys
import rospy 
from offloadable_face_recognition.msg import MotorCommand
import time

class Motor_Controller:

	def __init__(self):

		self.PORT_NUMBER = 5005
		self.DATA_SIZE = 1024
		self.socket = None
		self.MOTOR_COMMANDS = "motor_commands"
		self.queue_size = 1

		motor_server_ip = raw_input("Please enter the IP address of the motor_server: ")
		
		print "Initialising connection..."
		self.initialise_connection(motor_server_ip)

		self.motor_commands = rospy.Subscriber(self.MOTOR_COMMANDS, MotorCommand, self.motor_command_listener, queue_size=self.queue_size)

	def motor_command_listener(self, motor_command):
		print "motor_controller: command recieved " + str(motor_command.command)
		self.socket.send(motor_command.command)


	def initialise_connection(self, ip_addr):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.connect((ip_addr, self.PORT_NUMBER))


	def test(self):
		while True:
			self.socket.send("yaw_right")
			time.sleep(1)

def main(self):
	mc = Motor_Controller()
	mc.test()

if __name__ == '__main__':
	main(sys.argv)
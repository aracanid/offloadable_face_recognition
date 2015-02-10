#!/usr/bin/env python

import socket
import sys
from ev3.ev3dev import Motor

class motor_controller():

	self.PORT_NUMBER = 5005
	self.DATA_SIZE = 1024
	self.socket = None

	self.right_track_motor=Motor(port=Motor.PORT.A)
	self.left_track_motor=Motor(port=Motor.PORT.B)
	self.servo_arm_pitch_motor=Motor(port=Motor.PORT.C)
	self.servo_arm_roll_motor=Motor(port=Motor.PORT.D)

	def setup(self):
		controller_ip = raw_input("Please enter the IP address of the controlling pc: ")
		
		print "Setting up connection..."
		self.initialise_connection(controller_ip, self.PORT_NUMBER)
		
		print "Initialising Motors..."
		self.initialise_motors()


	def initialise_connection(self, ip_addr, port):
		hostname = socket.gethostbyname( '0.0.0.0' )
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.bind((hostname, PORT))


	def initialise_motors(self):
		set_up_motor(self.right_track_motor)
		set_up_motor(self.left_track_motor)
		set_up_motor(self.servo_arm_pitch_motor)
		set_up_motor(self.servo_arm_roll_motor)

	def set_up_motor(self, motor):
	    motor.reset()
	    motor.position = 0
	    motor.regulation_mode = True
	    motor.position_mode = relative
	    motor.run_mode = position
	    motor.pulses_per_second_sp = 30
	    motor.hold_mode = on
	    motor.stop_mode = brake

	def process_input_command(self, data):


def main(self):

	setup()

	while True:
		(data,addr) = s.recvfrom(SIZE) #data size


if __name__ == '__main__':

	main(sys.argv)
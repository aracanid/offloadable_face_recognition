import socket
import sys
from ev3.ev3dev import Motor

class Ev3_Motor_Server:

	def __init__(self):

		self.PORT_NUMBER = 5005
		self.DATA_SIZE = 1024
		
		self.DEFAULT_PULSES_PER_SECOND = 100
		self.DEFAULT_POSITION_SP = 10
		self.DEFAULT_NEGATIVE_POSITION_SP = -(self.DEFAULT_POSITION_SP)
		self.DEFAULT_RUN_MODE = "position"
		self.DEFAULT_HOLD_MODE = "on"
		self.DEFAULT_STOP_MODE = "brake"
		self.DEFAULT_POSITION_MODE = "relative"
		self.DEFAULT_REGULATION_MODE = True
		self.MOTOR_STARTING_POSITION = 0

		self.YAW_LEFT = "yaw_left"
		self.YAW_RIGHT = "yaw_right"

		self.socket = None

		self.right_track_motor=Motor(port=Motor.PORT.A)
		self.left_track_motor=Motor(port=Motor.PORT.B)
		#self.servo_arm_pitch_motor=Motor(port=Motor.PORT.C)
		#self.servo_arm_roll_motor=Motor(port=Motor.PORT.D)

		print "Setting up connection..."
		self.initialise_connection()
		
		print "Initialising Motors..."
		self.initialise_motors()		

		print "Awaiting new connection..."

	def initialise_connection(self):
		hostname = socket.gethostbyname( '0.0.0.0' )
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.bind((hostname, self.PORT_NUMBER))
		self.socket.listen(5)

	def initialise_motors(self):
		self.set_up_motor(self.right_track_motor)
		self.set_up_motor(self.left_track_motor)
		#set_up_motor(self.servo_arm_pitch_motor)
		#set_up_motor(self.servo_arm_roll_motor)

	def set_up_motor(self, motor):
	    motor.reset()
	    motor.position = self.MOTOR_STARTING_POSITION
	    motor.regulation_mode = self.DEFAULT_REGULATION_MODE
	    motor.position_mode = self.DEFAULT_POSITION_MODE
	    motor.run_mode = self.DEFAULT_RUN_MODE
	    motor.pulses_per_second_sp = self.DEFAULT_PULSES_PER_SECOND
	    motor.hold_mode = self.DEFAULT_HOLD_MODE
	    motor.stop_mode = self.DEFAULT_STOP_MODE
	    motor.position_sp = self.DEFAULT_POSITION_SP

	def yaw_left(self):
		self.left_track_motor.position_sp = self.DEFAULT_NEGATIVE_POSITION_SP
		self.right_track_motor.position_sp = self.DEFAULT_POSITION_SP
		self.left_track_motor.start()
		self.right_track_motor.start()

	def yaw_right(self):
		self.left_track_motor.position_sp = self.DEFAULT_POSITION_SP
		self.right_track_motor.position_sp = self.DEFAULT_NEGATIVE_POSITION_SP
		self.left_track_motor.start()
		self.right_track_motor.start()

	def process_input_command(self):

		conn, addr = self.socket.accept()
		print "Connection address: ", addr

		while True:
			data = conn.recv(self.DATA_SIZE) #data size
		
			if data == self.YAW_LEFT:
				self.yaw_left()		
			elif data == self.YAW_RIGHT:
				self.yaw_right()
			else:
				print "Unknown command received!"

		conn.close()

def main(self):
	try:
		ev3ms = Ev3_Motor_Server()
		ev3ms.process_input_command()
	except error:
		print "Connection Error: " + error

if __name__ == '__main__':
	main(sys.argv)
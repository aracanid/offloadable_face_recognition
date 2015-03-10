import socket
import sys
from ev3.ev3dev import Motor
from offloadable_face_recognition.errors import MotorError
import struct
import threading

class Ev3_Motor_Server:

	def __init__(self):

		self.PORT_NUMBER = 5004
		self.DATA_SIZE = 1024
		
		self.DEFAULT_PULSES_PER_SECOND = 0
		self.DEFAULT_HOLD_MODE = "on"
		self.DEFAULT_STOP_MODE = "brake"
		self.DEFAULT_REGULATION_MODE = True
		self.MOTOR_STARTING_POSITION = 0

		self.system_on = True

		self.YAW_LEFT = "yaw_left "
		self.YAW_RIGHT = "yaw_right"

		self.socket = None

		self.motor_lock = threading.Lock()
		self.unpacker = struct.Struct('f f')

		self.right_track_motor=Motor(port=Motor.PORT.A)
		self.left_track_motor=Motor(port=Motor.PORT.B)
		#self.servo_arm_pitch_motor=Motor(port=Motor.PORT.C)
		#self.servo_arm_roll_motor=Motor(port=Motor.PORT.D)

		self.camera_fov = 320
		self.camera_av = self.camera_fov/2
		self.face_x
		self.face_y


		self.kp = 10 					# Constant for the proportional controller
		self.ki = 1 					# Constant for the integral controller
		self.kd = 100 					# Constant for the differential controller
		self.offset = self.camera_av 	# Offtset used to calculate error
		self.tp = 0 					# Turning power of motors when there is no error
		self.previous_dt = 0
		self.integral = 0 	
		self.previous_err = 0 
		self.derivative = 0 

		self.coordinates_time_recv = 0
		self.timeout = 3 #seconds

		self.face_coordinates_lock = threading.Lock()
		self.system_on_lock = threading.Lock()
		self.update_coordinates_thread = threading.Thread(target = update_face_coordinates)
		self.update_coordinates_thread.start()



		print "Setting up connection.\n"
		self.initialise_connection()
		
		print "Initialising Motors..\n"
		self.initialise_motors()		

		print "Awaiting new connection...\n"

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
	    motor.pulses_per_second_sp = self.DEFAULT_PULSES_PER_SECOND
	    motor.hold_mode = self.DEFAULT_HOLD_MODE
	    motor.stop_mode = self.DEFAULT_STOP_MODE

	def current_time(self):
		return time.time()/1000 # In seconds

	def set_face_coordinates(self, x, y):
		with self.face_coordinates_lock:
			self.x = x
			self.y = y

	def get_face_coordinates(self):
		return self.x, self.y

	def set_system_on(self, state):
		with self.system_on_lock:
			self.system_on = state

	def get_system_on(self):
		return self.system_on

	def calculate_x_axis_pid(self):

	  	face_x, face_y = self.get_face_coordinates()
	  	dt = current_time() - self.previous_dt
	  	error = face_x - self.offset
	  	self.integral = integral + (error*dt)
	  	self.derivative = (error - self.previous_err)/dt

	  	turning_power = (self.kp*error) + (self.ki*integral) + (self.kd*derivative)
	  	left_motor_power = self.tp + turning_power 		# These depend on the inital 
	  	right_motor_power = self.tp - turning_power		# orientation of the motors

	  	self.right_track_motor.pulses_per_second_sp = right_motor_power
	  	self.left_track_motor.pulses_per_second_sp = left_motor_power

	  	self.previous_dt = dt
	  	self.previous_err = error

	 def stop_x_axis_motors(self):
	 	self.right_track_motor.stop()
	 	self.left_track_motor.stop()

	def motor_control(self):
		self.previous_dt = self.current_time()
		while get_system_on():
			try:
				if self.coordinates_time_recv > 0:
		  			calculate_x_axis_pid()
		  		else:
		  			self.previous_dt = time.time()
		  			self.stop_x_axis_motors()

	  		except (KeyboardInterrupt, SystemExit):
			  	print "Exiting motor controller!"
			  	self.stop_x_axis_motors()
			  	set_system_on(False)
			    raise	


	def update_face_coordinates(self):
		conn, addr = self.socket.accept()
		self.coordinates_time_recv = -1
		print "Connection address: ", addr

		while get_system_on():
			try:
				data = conn.recv(self.unpacker.size) #data size
				if not data:
					if self.current_time() > (self.coordinates_time_recv*self.timeout) and (self.coordinates_time_recv > -1):
						self.coordinates_time_recv = -1
						raise MotorError.CoordinatesTimeOut()
				else:
					self.face_x, face_y = self.unpacker.unpack(data)
					self.coordinates_time_recv = self.current_time()
					self.set_face_coordinates(face_x, face_y)
					print "Face coordinates recieved at: " + self.coordinates_time_recv + "s,  Position: ("self.face_x + "," + self.face_y + ")\n"

			except MotorError.CoordinatesTimeOut, e:
				self.coordinates_time_recv = -1
		  		print "ERROR: No new coordinates recieved!\nThe current coordinates are no longer valid, stopping motors!\n" + e

		 conn.close()

def main(self):
	ev3ms = Ev3_Motor_Server()
	ev3ms.motor_control()

if __name__ == '__main__':
	main(sys.argv)
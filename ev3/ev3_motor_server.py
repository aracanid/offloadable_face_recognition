import socket
import sys
from ev3.ev3dev import Motor
import time
#from offloadable_face_recognition.errors import MotorError
import struct
import threading

class MotorError(Exception):
	pass

class CoordinatesTimeOut(MotorError):
	pass

class Ev3_Motor_Server:

	def __init__(self):

		self.PORT_NUMBER = 5004
		self.DATA_SIZE = 1024
		
		self.DEFAULT_PULSES_PER_SECOND = 0
		self.DEFAULT_HOLD_MODE = "on"
		self.DEFAULT_STOP_MODE = "brake"
		self.DEFAULT_REGULATION_MODE = True
		self.MOTOR_STARTING_POSITION = 0

		self.system_on = False
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
		self.face_x = 0
		self.face_y = 0


		self.kp = 1					# Constant for the proportional controller
		self.ki = 0 					# Constant for the integral controller
		self.kd = 0 					# Constant for the differential controller
		self.offset = self.camera_av 	# Offtset used to calculate error
		self.tp = 0 					# Turning power of motors when there is no error
		self.previous_dt = 0
		self.integral = 0 	
		self.previous_err = 0 
		self.derivative = 0 

		self.coordinates_time_recv = 0
		self.timeout = 1 #seconds

		self.face_coordinates_lock = threading.Lock()
		self.system_on_lock = threading.Lock()
		self.update_coordinates_thread = threading.Thread(target = self.update_face_coordinates)
		self.update_coordinates_thread.start()

		print "Setting up connection.\n"
		self.initialise_connection()
		
		print "Initialising Motors..\n"
		self.initialise_motors()		

		print "Awaiting new connection...\n"
		self.update_face_coordinates()

	def initialise_connection(self):
		hostname = socket.gethostbyname( '0.0.0.0' )
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.bind((hostname, self.PORT_NUMBER))
		self.socket.listen(1)

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
		dt = self.current_time() - self.previous_dt
		error = face_x - self.offset
		self.integral = self.integral + (error*dt)
		self.derivative = (error - self.previous_err)/dt

		turning_power = (self.kp*error) + (self.ki*self.integral) + (self.kd*self.derivative)
		left_motor_power = self.tp + turning_power 		# These depend on the inital 
		right_motor_power = self.tp - turning_power		# orientation of the motors
		print str(turning_power)
		self.right_track_motor.pulses_per_second_sp = right_motor_power
		self.left_track_motor.pulses_per_second_sp = left_motor_power

		self.previous_dt = dt
		self.previous_err = error

	def stop_x_axis_motors(self):
		self.right_track_motor.stop()
		self.left_track_motor.stop()

	def start_x_axis_motors(self):
		self.right_track_motor.start()
		self.left_track_motor.start()

	def motor_control(self):
		self.previous_dt = self.current_time()
		try:
			if self.coordinates_time_recv > 0:
				self.calculate_x_axis_pid()
				self.start_x_axis_motors()
			else:
				self.previous_dt = time.time()
				self.stop_x_axis_motors()

		except (KeyboardInterrupt, SystemExit):
			#self.cleanup()
			raise

	def update_face_coordinates(self):
		conn, addr = self.socket.accept()
		print conn, addr
		self.coordinates_time_recv = -1
		print "Connection address: ", addr
		self.set_system_on(True)

		while self.get_system_on():
			try:
				data = conn.recv(self.unpacker.size) #data size
				if not data:
					if self.current_time() > (self.coordinates_time_recv*self.timeout) and (self.coordinates_time_recv > -1):
						self.coordinates_time_recv = -1
						raise CoordinatesTimeOut()
				else:
					face_x, face_y = self.unpacker.unpack(data)
					print face_x, face_y
					self.coordinates_time_recv = self.current_time()
					self.set_face_coordinates(face_x, face_y)
					print "Face coordinates recieved at: " + str(self.coordinates_time_recv) + "s,  Position: (" + str(face_x) + "," + str(face_y) + ")\n"

				self.motor_control()

			except CoordinatesTimeOut, e:
				self.coordinates_time_recv = -1
				self.stop_x_axis_motors()
				print "ERROR: No new coordinates recieved!\nThe current coordinates are no longer valid, stopping motors!\n" + e
			except KeyboardInterrupt, SystemExit:
				self.cleanup()
				conn.close()

		conn.close()

	def cleanup(self):
		print "Exiting motor controller!"
		self.stop_x_axis_motors()
		self.set_system_on(False)

def main(self):
	ev3ms = Ev3_Motor_Server()

if __name__ == '__main__':
	main(sys.argv)
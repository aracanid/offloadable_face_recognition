import socket
import ev3.ev3dev
import time
from ev3.ev3dev import Motor


rightTrackMotor=Motor(port=Motor.PORT.A)
leftTrackMotor=Motor(port=Motor.PORT.B)
servoArmPitchMotor=Motor(port=Motor.PORT.C)
servoArmRollMotor=Motor(port=Motor.PORT.D)

print "Motors initialised."

def setUp(motor):
        motor.reset()
        motor.regulation_mode = True
        motor.pulses_per_second_sp = 30

setUp(rightTrackMotor)
setUp(leftTrackMotor)
setUp(servoArmPitchMotor)
setUp(servoArmRollMotor)

print "Motors set up."
#def yaw(direction):
        #left/right

#def roll(direction):
        #left/right

#def pitch(direction):
        #up/down

UDP_IP = "10.42.0.3"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP,UDP_PORT))

print "Robot awaiting commands..."

while True:
        data, addr = sock.recvfrom(1024)

        if(data=="right"):
                leftTrackMotor.pulses_per_second_sp = 100
                rightTrackMotor.pulses_per_second_sp = -100
                leftTrackMotor.start()
                rightTrackMotor.start()
		elif(data=="left"):
                leftTrackMotor.pulses_per_second_sp = -100
                rightTrackMotor.pulses_per_second_sp = 100
                leftTrackMotor.start()
                rightTrackMotor.start()
        elif(data=="pitchup"):
                servoArmPitchMotor.pulses_per_second_sp = -100
                servoArmPitchMotor.start()
        elif(data=="pitchdown"):
                servoArmPitchMotor.pulses_per_second_sp = 100
                servoArmPitchMotor.start()
        elif(data=="rollleft"):
                servoArmRollMotor.pulses_per_second_sp = -60
                servoArmRollMotor.start()
        elif(data=="rollright"):
                servoArmRollMotor.pulses_per_second_sp = 60
                servoArmRollMotor.start()
        elif(data=="killall"):
                rightTrackMotor.stop()
                leftTrackMotor.stop()
                servoArmPitchMotor.stop()
                servoArmRollMotor.stop()
        #time.sleep(2)
        #rightTrackMotor.stop()
        #leftTrackMotor.stop()
        #servoArmPitchMotor.stop()
        #servoArmRollMotor.stop()



import socket
import time
import termios, fcntl, sys, os
    
UDP_IP = "10.42.0.3"
UDP_PORT = 5005
MESSAGE = "Hello, World!"
   
print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "Ready..."
  
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fd = sys.stdin.fileno()

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

try:
    while 1:
        time.sleep(1)
        try:
            c = sys.stdin.read(1)
            if(c=="w"):
              sock.sendto("pitchup", (UDP_IP, UDP_PORT))
              print "ServoArm Pitch Up"
            elif(c=="s"):
              sock.sendto("pitchdown", (UDP_IP, UDP_PORT))
              print "ServoArm Pitch Down"
            elif(c=="a"):
              sock.sendto("left", (UDP_IP, UDP_PORT))
              print "Tracks Yaw Left"
            elif(c=="d"):
              sock.sendto("right", (UDP_IP, UDP_PORT))
              print "Tracks Yaw Right"
            elif(c=="q"):
              sock.sendto("rollleft", (UDP_IP, UDP_PORT))
              print "Camera Roll Left"
            elif(c=="e"):
              sock.sendto("rollright", (UDP_IP, UDP_PORT))
              print "Camera Roll Right"
            elif(c=="k"):
              sock.sendto("killall", (UDP_IP,UDP_PORT))
            else:
              sock.sendto("killall", (UDP_IP,UDP_PORT))
        except IOError: pass
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)



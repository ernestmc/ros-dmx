import serial
import termios
import fcntl
import time

import rospy
from geometry_msgs.msg import Twist

global datum

def receive(data):
  global datum
  print data
  datum = (data.linear.x + 1) / 2 * 255
  print datum
  

global pub
rospy.init_node('cmdvel2turtlebot', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, receive)

TIOCSBRK = getattr(termios, 'TIOCSBRK', 0x5427)
TIOCCBRK = getattr(termios, 'TIOCCBRK', 0x5428)

BAUD_RATE = 250e3

comport = serial.Serial("/dev/ttyUSB0", baudrate=BAUD_RATE, stopbits=serial.STOPBITS_TWO)

data = chr(0) + chr(128) + chr(0) + chr(128) + chr(0) * 508

a = 0
d = 1
datum = 0
while 1:
  #a = a + d
  #if a == 128 or a == 0:
  #  d = -d
  a = int(datum)
  if a > 255:
    a = 255
  if a < 0:
    a = 0
  print a
  data = chr(0) + chr(a) + chr(0) * 510
  # Control manual del break
  fcntl.ioctl(comport.fd, TIOCSBRK)
  time.sleep(100/1e6)
  fcntl.ioctl(comport.fd, TIOCCBRK)
  time.sleep(100/1e6)
  comport.write(data)
  #time.sleep(0.01)
  rospy.sleep(0.01)

comport.close()

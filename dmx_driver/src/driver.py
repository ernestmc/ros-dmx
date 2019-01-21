#! /usr/bin/env python

import serial
import termios
import fcntl
import time
import rospy
from dmx_msgs.msg import DmxFrame


"""
Class to control and communicate with the DMX hardware.
"""
class DmxDriver(object):

  def __init__(self, com_port="/dev/ttyUSB0"):
    """
    Initializer
    @param com_port: identifier of the serial por to use
    """
    self.rate = rospy.Rate(50)
    self.BAUD_RATE = 250000
    try:
      self.serial_device = serial.Serial(com_port, baudrate=self.BAUD_RATE, stopbits=serial.STOPBITS_TWO)
    except serial.SerialException, e:
      rospy.logerr(str(e))
    self.TIOCSBRK = getattr(termios, 'TIOCSBRK', 0x5427)
    self.TIOCCBRK = getattr(termios, 'TIOCCBRK', 0x5428)
    self.dmx_frame_subscriber = rospy.Subscriber('/dmx_frame', DmxFrame, self.cb_dmx_frame, queue_size=10)

  def __del__(self):
    """
    Remove device on exit
    """
    self.serial_device.close()

  def run(self):
    """
    Main loop.
    """
    while not rospy.is_shutdown():
      self.rate.sleep()

  def send_frame(self, frame):
    """
    Send a DMX frame.
    @param frame: DMX frame to send, a list of bytes
    """
    # Convert frame to string
    # Channel 0 is the break condition
    serial_data = chr(0)
    for i in frame:
      serial_data += i
    # Manually emulate the serial break condition
    fcntl.ioctl(self.serial_device.fd, self.TIOCSBRK)
    time.sleep(100 / 1e6)
    fcntl.ioctl(self.serial_device.fd, self.TIOCCBRK)
    time.sleep(100 / 1e6)
    self.serial_device.write(serial_data)

  def cb_dmx_frame(self, data):
    """
    Callback to receive a dmx frame.
    @param data: Frame message.
    """
    # type: (DmxFrame) -> None
    self.send_frame(data.frame)


if __name__ == '__main__':
  rospy.init_node("dmx_driver")
  node = DmxDriver()
  node.run()

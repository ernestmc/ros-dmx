#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from dmx_driver.srv import SetChannel, SetChannelResponse
from dmx_msgs.msg import DmxFrame


class DmxManager(object):
  def __init__(self):
    self.rate = rospy.Rate(50)
    self.dmx_frame = [0] * 512
    self.position_subscriber = rospy.Subscriber("/position", Point, self.position_cb)
    self.dmx_frame_publisher = rospy.Publisher('/dmx_frame', DmxFrame, queue_size=10)
    self.services = [
      rospy.Service("set_channel", SetChannel, self.set_channel_srv_cb)
    ]

  def zero_dmx_frame(self):
    """
    Put all channels of the DMX frame to zero.
    :return: None
    """
    for i in range(512):
      self.dmx_frame[i] = 0

  def set_channel_srv_cb(self, req):
    self.set_channel(req.channel, req.value)
    return SetChannelResponse()

  def set_channel(self, channel, value):
    """
    Set a channel to a specific value.
    :param channel: A channel value in the range [1; 512]
    :param value: A value in the range [0; 255]
    :return: None
    """
    channel -= 1
    if channel > 511:
      channel = 511
    elif channel < 0:
      channel = 0
    self.dmx_frame[channel] = value

  def position_cb(self, data):
    x = 128 - (data.x - 320) * 20 / 320
    y = 128 - (data.y - 240) * 40 / 240 - 64
    z = 255 - data.z * 4
    if z < 0:
      z = 0
    if z > 255:
      z = 255
    self.set_dmx(int(x), int(y), int(z))

  def set_dmx(self, x, y, z):
    print x, y, z
    self.set_channel(1, x)
    self.set_channel(3, y)
    self.set_channel(6, z / 4)

  def run(self):
    # open shutter
    self.set_channel(7, 255)
    # dimmer
    self.set_channel(6, 10)
    while not rospy.is_shutdown():
      self.publish_dmx_frame()
      self.rate.sleep()

  def publish_dmx_frame(self):
    dmx_frame = DmxFrame()
    dmx_frame.frame = self.dmx_frame
    self.dmx_frame_publisher.publish(dmx_frame)


if __name__ == '__main__':
  rospy.init_node("dmx_manager")
  node = DmxManager()
  node.run()
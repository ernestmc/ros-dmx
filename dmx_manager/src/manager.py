#! /usr/bin/env python

import rospy
from dmx_msgs.srv import SetChannel, SetChannelResponse, GetChannel, GetChannelResponse
from dmx_msgs.msg import DmxFrame


"""
Class that generate a 50Hz DMX frame. Also provides services to manipulate channel information in the DMX frame.
"""
class DmxManager(object):
  def __init__(self):
    self.rate = rospy.Rate(50)
    self.dmx_frame = [0] * 512
    self.dmx_frame_publisher = rospy.Publisher('/dmx_frame', DmxFrame, queue_size=10)
    self.services = [
      rospy.Service("set_channel", SetChannel, self.set_channel_srv_cb),
      rospy.Service("get_channel", GetChannel, self.get_channel_srv_cb)
    ]

  def zero_dmx_frame(self):
    """
    Put all channels of the DMX frame to zero.
    """
    for i in range(512):
      self.dmx_frame[i] = 0

  def set_channel_srv_cb(self, req):
    """
    Callback to process set channel service requests.
    @param req: Request message
    """
    self.set_channel(req.channel, req.value)
    return SetChannelResponse()

  def get_channel_srv_cb(self, req):
    """
    Callback to process get channel service requests.
    @param req: Request message
    """
    resp = GetChannelResponse()
    resp.value = self.get_channel(req.channel)
    return resp

  def set_channel(self, channel, value):
    """
    Set a channel to a specific value.
    @param channel: A channel value in the range [1; 512]
    @param value: A value in the range [0; 255]
    """
    channel -= 1
    if channel >= 0 and channel < 512:
      self.dmx_frame[channel] = value

  def get_channel(self, channel):
    """
    Get the current value of a channel.
    @param channel: A channel value in the range [1; 512]
    @return: 8 bit value for that channel
    """
    channel -= 1
    if channel >= 0 and channel < 512:
      return self.dmx_frame[channel]
    else:
      return None

  def run(self):
    """
    Main loop.
    """
    # open shutter
    self.set_channel(7, 255)
    # dimmer
    self.set_channel(6, 10)
    while not rospy.is_shutdown():
      self.publish_dmx_frame()
      self.rate.sleep()

  def publish_dmx_frame(self):
    """
    Publish the DMX frame message.
    """
    dmx_frame = DmxFrame()
    dmx_frame.frame = self.dmx_frame
    self.dmx_frame_publisher.publish(dmx_frame)


if __name__ == '__main__':
  rospy.init_node("dmx_manager")
  node = DmxManager()
  node.run()
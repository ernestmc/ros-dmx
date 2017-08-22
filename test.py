import rospy
from geometry_msgs.msg import Point
from dmx_driver.srv import SetChannel, SetChannelRequest


class Test(object):
  def __init__(self):
    self.rate = rospy.Rate(10)
    self.position_subscriber = rospy.Subscriber("/position", Point, self.position_cb)
    self.set_channel = rospy.ServiceProxy("/set_channel", SetChannel)

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
    self.set_channel(SetChannelRequest(1, x))
    self.set_channel(SetChannelRequest(3, y))
    self.set_channel(SetChannelRequest(6, z / 4))

  def run(self):
    # open shutter
    self.set_channel(7, 255)
    # dimmer
    self.set_channel(6, 10)
    while not rospy.is_shutdown():
      self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node("test")
  node = Test()
  node.run()
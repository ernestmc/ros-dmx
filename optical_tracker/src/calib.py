#!/usr/bin/env python

import rospy
import pygame
import math
from pygame import key
from geometry_msgs.msg import Point
from dmx_driver.srv import SetChannel, SetChannelRequest
from widgets import PanGauge, TiltGauge

class Calibration(object):
    def __init__(self):
        pygame.init()
        display_info = pygame.display.Info()
        pygame.display.set_mode([display_info.current_w / 2, display_info.current_h / 2])
        self.rate = rospy.Rate(10)
        self.latest_point = Point()
        self.widgets = {
            'pan': PanGauge(200, 100),
            'tilt': TiltGauge(100, 200)
        }
        self.screen = pygame.display.get_surface()
        #self.position_subscriber = rospy.Subscriber("/position", Point, self.position_cb)
        #self.set_channel = rospy.ServiceProxy("/set_channel", SetChannel)

    def calibrate(self):
        pan = 0
        tilt = 0
        print "Calibrating..."
        inc_pan = 0.05
        inc_tilt = 0.05
        while not rospy.is_shutdown():
            pygame.event.pump()
            keys = key.get_pressed()
            pan = pan + inc_pan
            if pan > math.pi or pan < 0:
                inc_pan = -inc_pan
            tilt = tilt + inc_tilt
            if tilt > math.pi or tilt < 0:
                inc_tilt = -inc_tilt
            self.screen.fill(pygame.Color(0, 0, 0))
            self.widgets['pan'].set_pan(pan)
            self.widgets['pan'].draw(self.screen, 0, 0)
            self.widgets['tilt'].set_tilt(tilt)
            self.widgets['tilt'].draw(self.screen, 200, 0)
            pygame.display.flip()
            if keys[pygame.K_ESCAPE]:
                return
            self.rate.sleep()

    def run(self):
        print "Starting calibration..."
        while not rospy.is_shutdown():
            pygame.event.pump()
            keys = key.get_pressed()
            if keys[pygame.K_c]:
                self.calibrate()
            self.rate.sleep()

    def position_cb(self, point):
        assert(isinstance(point, Point))
        self.latest_point = point


if __name__ == '__main__':
  rospy.init_node("test_calibration")
  node = Calibration()
  node.run()

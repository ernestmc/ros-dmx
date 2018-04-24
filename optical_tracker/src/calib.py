#!/usr/bin/env python

import rospy
import pygame
import math
import numpy as np
from pygame import key
from geometry_msgs.msg import Point
from dmx_msgs.srv import GetChannel, GetChannelRequest
from widgets import PanGauge, TiltGauge

DEGREES_PER_PAN = 540.0 / 255
DEGREES_PER_TILT = 270.0 / 255
RADIANS_PER_PAN = DEGREES_PER_PAN * math.pi / 180
RADIANS_PER_TILT = DEGREES_PER_TILT * math.pi / 180

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
        self.get_channel = rospy.ServiceProxy("/get_channel", GetChannel)

    def calibrate(self):
        print "Calibrating..."
        pass

    def run(self):
        print "\nCalibration program.\n"
        print "Press 'c' to enter a calibration point.\n\n"
        while not rospy.is_shutdown():
            # update gauges
            norm_pan_rad = math.radians(self.normalize_pan(self.get_pan()))
            norm_tilt_rad = math.radians(self.normalize_tilt(self.get_tilt()))
            self.update_display(norm_pan_rad, norm_tilt_rad)
            pygame.event.pump()
            keys = key.get_pressed()
            # pan = pan + inc_pan
            # if pan > math.pi or pan < 0:
            #     inc_pan = -inc_pan
            # tilt = tilt + inc_tilt
            # if tilt > math.pi or tilt < 0:
            #     inc_tilt = -inc_tilt
            pan = self.get_normalized_pan()
            tilt = self.get_normalized_tilt()
            self.screen.fill(pygame.Color(0, 0, 0))
            self.widgets['pan'].set_pan(pan)
            self.widgets['pan'].draw(self.screen, 0, 0)
            self.widgets['tilt'].set_tilt(tilt)
            self.widgets['tilt'].draw(self.screen, 200, 0)
            pygame.display.flip()
            if keys[pygame.K_ESCAPE]:
                return
            self.rate.sleep()

    def position_cb(self, point):
        assert(isinstance(point, Point))
        self.latest_point = point

    def get_normalized_pan(self):
        pan = self.get_channel(1).value * DEGREES_PER_PAN + 180
        return pan * math.pi / 180


if __name__ == '__main__':
  rospy.init_node("test_calibration")
  node = Calibration()
  node.run()

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
        self.calib_mat = []
        self.widgets = {
            'pan': PanGauge(200, 100),
            'tilt': TiltGauge(100, 200)
        }
        self.screen = pygame.display.get_surface()
        #self.position_subscriber = rospy.Subscriber("/position", Point, self.position_cb)
        self.get_channel = rospy.ServiceProxy("/get_channel", GetChannel)

    def calibrate(self):
        print "Calibrating..."
        x, position = self.calculate_transform_matrix(self.cal_xy_points, self.cal_pt_points)
        print "Calibrated matrix: %s" % x
        print "Position: %s" % position

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

    def normalize_pan(self, pan):
        norm_pan = - pan * DEGREES_PER_PAN + 360
        return norm_pan

    def normalize_tilt(self, tilt):
        norm_tilt = tilt * DEGREES_PER_TILT - 45
        return norm_tilt

    def denormalize_pan(self, normalized_pan):
        return (normalized_pan * 180 / math.pi - 180) / DEGREES_PER_PAN

    def denormalize_tilt(self, normalized_tilt):
        return normalized_tilt * 180 / math.pi

    def update_display(self, pan, tilt):
        self.screen.fill(pygame.Color(0, 0, 0))
        self.widgets['pan'].set_pan(pan)
        self.widgets['pan'].draw(self.screen, 0, 0)
        self.widgets['tilt'].set_tilt(tilt)
        self.widgets['tilt'].draw(self.screen, 200, 0)
        pygame.display.flip()

    def add_cal_point(self, x, y, pan, tilt):
        self.cal_xy_points.append((x, y))
        self.cal_pt_points.append((pan, tilt))
        print "Added calibration point #%s -- xy:%s pan/tilt:%s" % \
              (len(self.cal_xy_points), self.cal_xy_points[-1], self.cal_pt_points[-1])

    def is_point_valid(self, x, y, pan, tilt):
        """
        Indicates if the point pairs are valid for calibration.
        :param x: x coordinate
        :param y: y coordinate
        :param pan: horizontal pan angle
        :param tilt: vertical tilt angle
        :return: True if points are valid, False otherwise
        """
        tolerance = 0.0001
        #if np.abs(pan) < tolerance and np.abs(tilt) < tolerance:
        #    return False
        return True

    def calculate_transform_matrix(self, xy, pt):
        """
        Given a list of (x, y) coordinates and the corresponding (pan, tilt) angles, estimate the transformation matrix.
        Uses least square to compute the best fitting matrix for the value pairs.
        :param xy: a list of coordinate tuples [(x, y)]
        :param pt: a list of pan and tilt angles in radians [(pan, tilt)]. Angles should be in the range (-pi/2, pi/2)
        :return: a transformation matrix that converts (x, y) coordinates into (pan, tilt) angles.
        """
        xy = np.array(xy)
        pt = np.array(pt)

        P = pt[:, 0]  # pan vector
        T = pt[:, 1]  # tilt vector
        gx = np.tan(P)
        gy = np.tan(T)

        rx = xy[:, 0]  # X vector
        ry = xy[:, 1]  # Y vector

        A = np.mat([rx, ry, np.ones(len(rx))])
        A = np.transpose(A)

        b = np.mat([gx, gy])
        b = np.transpose(b)

        x = np.linalg.lstsq(A, b)[0]
        cz0 = (x.item((0, 0)) + x.item((1, 1))) / 2
        cx0 = -x.item((2, 0)) * cz0
        cy0 = -x.item((2, 1)) * cz0
        position = np.array([cx0, cy0, cz0]).transpose()
        return x, position


if __name__ == '__main__':
  rospy.init_node("test_calibration")
  node = Calibration()
  node.run()

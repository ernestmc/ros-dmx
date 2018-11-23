#!/usr/bin/env python

import rospy
import pygame
import math
import time
import numpy as np
from pygame import key
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from dmx_msgs.srv import GetChannel, SetChannel
from widgets import PanGauge, TiltGauge
from threading import RLock

PAN_CHANNEL = 1
PAN_INCREMENT = 1
TILT_CHANNEL = 3
TILT_INCREMENT = 1
DIMMER_CHANNEL = 6
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
        self.cal_xy_points = []
        self.cal_pt_points = []
        self.latest_point = Point()
        self.last_joy = Joy()
        self.calib_mat = None
        self.widgets = {
            'pan': PanGauge(200, 100),
            'tilt': TiltGauge(100, 200)
        }
        self.screen = pygame.display.get_surface()
        self.position_subscriber = rospy.Subscriber("/position", Point, self.position_cb)
        self.joystick_subscriber = rospy.Subscriber("/joy", Joy, self.joystick_cb)
        self.get_channel = rospy.ServiceProxy("/get_channel", GetChannel)
        self.set_channel = rospy.ServiceProxy("/set_channel", SetChannel)
        self.lock = RLock()
        self.panning = 0
        self.tilting = 0
        self.reset_pose()

    def calibrate(self):
        print "Calibrating..."
        x, position = self.calculate_transform_matrix(self.cal_xy_points, self.cal_pt_points)
        self.calib_mat = x
        print "Calibrated matrix: %s" % x
        print
        print "Position: %s" % position
        print

    def run(self):
        print "\nCalibration program.\n"
        print "Press 'c' to enter a calibration point.\n\n"
        while not rospy.is_shutdown():
            # update gauges
            pan_rad = math.radians(self.pan_to_deg(self.get_pan()))
            tilt_rad = math.radians(self.tilt_to_deg(self.get_tilt()))
            self.update_display(pan_rad, tilt_rad)
            pygame.event.pump()
            keys = key.get_pressed()
            if keys[pygame.K_c] or (len(self.last_joy.buttons) > 0 and self.last_joy.buttons[0]):
                point = self.latest_point
                if self.is_point_valid(point.x, point.y, pan_rad, tilt_rad):
                    self.add_cal_point(point.x, point.y, pan_rad, tilt_rad)
                time.sleep(1)
            if keys[pygame.K_x] or (len(self.last_joy.buttons) > 8 and self.last_joy.buttons[9]):
                self.calibrate()
                time.sleep(1)
            if keys[pygame.K_l] or (len(self.last_joy.buttons) > 7 and self.last_joy.buttons[8]):
                self.calib_mat = None
                self.cal_pt_points = []
                self.cal_xy_points = []
                time.sleep(1)
            if self.panning:
                self.increment_channel(PAN_CHANNEL, self.panning)
            if self.tilting:
                self.increment_channel(TILT_CHANNEL, self.tilting)

            if self.calib_mat is not None:
                pt = self.transform_xy_to_pt(self.latest_point.x, self.latest_point.y)
                pt = np.rad2deg(pt)
                pt = pt.tolist()[0]
                print pt
                pan_estimate = self.deg_to_pan(pt[0])
                tilt_estimate = self.deg_to_tilt(pt[1])
                print pan_estimate
                print tilt_estimate
                self.set_channel(PAN_CHANNEL, self.clip_byte(pan_estimate))
                self.set_channel(TILT_CHANNEL, self.clip_byte(tilt_estimate))

            # check transforming calibrated points
            if self.calib_mat is not None:
                for i in range(len(self.cal_xy_points)):
                    xy = self.cal_xy_points[i]
                    pt = self.cal_pt_points[i]
                    cpt = self.transform_xy_to_pt(xy[0], xy[1])
                    print "Calibration point xy %s pt %s is converted to pan/tilt %s" % (xy, pt, cpt)
                    print
            self.rate.sleep()

    def reset_pose(self):
        initial_pan = self.deg_to_pan(0)
        initial_tilt = self.deg_to_tilt(0)
        self.set_channel(DIMMER_CHANNEL, 100)
        self.set_channel(PAN_CHANNEL, initial_pan)
        self.set_channel(TILT_CHANNEL, initial_tilt)

    def position_cb(self, point):
        # type: (Point)->None
        self.latest_point = point

    def joystick_cb(self, joy):
        # type: (Joy)->None
        self.last_joy = joy
        self.panning = joy.axes[0] * PAN_INCREMENT
        self.tilting = joy.axes[1] * TILT_CHANNEL


    def increment_channel(self, channel, increment):
        val = self.get_channel(channel).value
        val += increment
        if val < 0:
            val = 0
        if val > 255:
            val = 255
        self.set_channel(channel, val)

    def get_pan(self):
        return self.get_channel(1).value

    def get_tilt(self):
        return self.get_channel(3).value

    def pan_to_deg(self, pan):
        pan_deg = - pan * DEGREES_PER_PAN + 360
        return pan_deg

    def tilt_to_deg(self, tilt):
        tilt_deg = tilt * DEGREES_PER_TILT - 45
        return tilt_deg

    def deg_to_pan(self, pan_deg):
        pan = -(pan_deg - 360) / DEGREES_PER_PAN
        return pan

    def deg_to_tilt(self, tilt_deg):
        tilt = (tilt_deg + 45) / DEGREES_PER_TILT
        return tilt

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
        print "Added calibration point #%s -- xy:%s pan/tilt:%s\n" % \
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

    def transform_xy_to_pt(self, x, y):
        if self.calib_mat is not None:
            xy = np.array([x, y, 1])
            gxy = xy * self.calib_mat
            rad_angle = np.arctan(gxy)
            return rad_angle

    def clip_byte(self, a):
        b = math.floor(a)
        if b > 255:
            b = 255
        if b < 0:
            b = 0
        return b

if __name__ == '__main__':
  rospy.init_node("test_calibration")
  node = Calibration()
  node.run()

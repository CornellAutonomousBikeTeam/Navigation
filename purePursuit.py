#!/usr/bin/env python
# Copyright (C) 2015 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
"""
the :mod:`sbp.client.examples.simple` module contains a basic example of
reading SBP messages from a serial port, decoding BASELINE_NED messages and
printing them out.
"""

from sbp2.pyserial_driver import PySerialDriver
from sbp2.handler import Handler
from sbp2.framer import Framer
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
import argparse
import rospy
from std_msgs.msg import Float32, Float32MultiArray
import math

from path_following.histogram_grid import HistogramGrid
from path_following.polar_histogram import PolarHistogram
from path_following.pure_pursuit import PPPathPlanner

MAX_ANG = math.pi/6
roll = 0
SwiftRawMagMessageType=2306

def bike_state_callback(data):
    global roll
    roll = data.data[5]

def get_waypoints():
    return [(0, 0), (40, 15), (30, 50), (50, 50)]

def get_target_heading(planner, x, y, heading):
    # TODO: figure out how to use the pp_path_planner and the information
    # we get from ROS to get a heading and speed in the appropriate format
    pass

def main():
    rospy.init_node("nav_instr")
    rospy.Subscriber("bike_state", Float32MultiArray, bike_state_callback)
    nav_pub = rospy.Publisher('nav_instr', Float32, queue_size=10)
    tan_pub = rospy.Publisher('degrees', Float32, queue_size=10)
    waypoints = get_waypoints()
    pp_path_planner = PPPathPlanner(waypoints, lookahead_dist=10, max_lookahead_speed=None)
    # Open a connection to Piksi using the default baud rate (1Mbaud)
    with PySerialDriver("/dev/ttyUSB0", baud=115200) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            while True:
                try:
                    msg = source.wait(SwiftRawMagMessageType,0.1)
                    if msg is not None:
                        msg_x, msg_y, msg_z = msg.mag_x, msg.mag_y, msg.mag_z
                        print(msg_x, msg_y, msg_z)
                        # adjust for tilt
                        real_x = msg_x
                        real_y = math.cos(roll)*msg_y - math.sin(roll)*msg_z
                        real_z = math.sin(roll)*msg_y + math.cos(roll)*msg_z
                        tanangle = math.atan2(real_y,real_x)
                        degrees =math.degrees(tanangle)
                        # P controller
                        target_heading = get_target_heading(pp_path_planner, real_x, real_y, tanangle)
                        angle_diff = target_heading - tanangle
                        if abs(angle_diff) > math.pi:
                            angle_diff -= math.copysign(2*math.pi, angle_diff)
                        nav_instr = angle_diff
                        if nav_instr > MAX_ANG:
                            nav_instr = MAX_ANG
                        elif nav_instr < -MAX_ANG:
                            nav_instr = -MAX_ANG
                        nav_pub.publish(nav_instr)
                        tan_pub.publish(degrees)
                        print(degrees)
                    #i"i"for msg, metadata in source.filter(SBP_MSG_BASELINE_NED):
                    #    # Print out the N, E, D coordinates of the baseline
                    #    print("%.4f,%.4f,%.4f" % (msg.n * 1e-3, msg.e * 1e-3,
#                                                  msg.d * 1e-3))
                except KeyboardInterrupt:
                    pass

if __name__ == "__main__":
    main()


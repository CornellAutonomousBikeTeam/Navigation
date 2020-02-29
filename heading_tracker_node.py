#!/usr/bin/env python
import math
import rospy

from std_msgs.msg import Float32

MAX_ANG = math.pi/6

class HeadingTracker(object):
    def __init__(self):
        rospy.init_node("heading_tracker")
        self.pub = rospy.Publisher("/nav_instr", Float32, queue_size=10)
        self.target_heading = math.pi/2

    def heading_listener(self, data):
        if self.target_heading is None: return
        # P controller
        angle_diff = data.data - self.target_heading
        if abs(angle_diff) > math.pi:
            angle_diff -= math.copysign(2*math.pi, angle_diff)
        nav_instr = angle_diff
        if nav_instr > MAX_ANG:
            nav_instr = MAX_ANG
        elif nav_instr < -MAX_ANG:
            nav_instr = -MAX_ANG
        self.pub.publish(nav_instr)

    def target_listener(self, data):
        self.target_heading = data.data

    def go(self):
        rospy.Subscriber("/android/compass", Float32, self.heading_listener)
        rospy.Subscriber("/target_heading", Float32, self.target_listener)
        rospy.spin()

if __name__ == "__main__":
    HeadingTracker().go()

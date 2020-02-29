#!/usr/bin/env python
import rospy
import math

from util.location import global_to_local
from bike.msg import Location

import subprocess

class AndroidGps:
    def __init__(self):
        self.last_x = None
        self.last_y = None

    def make_location_message(self, lat, lng):
        x, y = global_to_local(lat, lng)
        if False:#(y - self.last_y) == 0 and (x - self.last_x) == 0:
            # we haven't moved
            return None
        yaw = math.atan2(y - self.last_y, x - self.last_x)
        speed = math.sqrt((y-self.last_y)**2 + (x-self.last_x)**2)
        self.last_x, self.last_y = x, y
        return Location(lat, lng, yaw, speed)

    def main(self):
        rospy.init_node("android_gps")
        pub_f = rospy.Publisher("/android/gps/fused", Location, queue_size=10)
        pub_g = rospy.Publisher("/android/gps/gps", Location, queue_size=10)

        ADB_BINARY = "adb"
        # for the old HTC Android 6 phone
        # ADB_BINARY = /home/pi/adb-1.0.31
        COMMAND = "%s shell dumpsys location" % ADB_BINARY

        LOC_TYPE = "gps" # fused for indoors, gps for outdoors, network or passive for the old HTC Android 6
        KEY_TEMPLATE = "    %s: Location[%s "
        KEY = KEY_TEMPLATE % (LOC_TYPE, LOC_TYPE)
        LOC_START = len(KEY)
        LOC_END = LOC_START + 20

        def loc_to_lat_lng(loc_type, loc_line):
            start = len(KEY_TEMPLATE % (loc_type, loc_type))
            end = start + 20
            return tuple(map(float, loc_line[start:end].split(",")))

        key_fused, key_gps = KEY_TEMPLATE % ("fused", "fused"), KEY_TEMPLATE % ("gps", "gps")

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            output = subprocess.check_output(COMMAND, shell=True)
            try:
                loc_line_gen = (line for line in output.split("\n") if line.startswith((key_fused, key_gps)))
                latlng_fused, latlng_gps = loc_to_lat_lng("fused", next(loc_line_gen)), loc_to_lat_lng("gps", next(loc_line_gen))

                pub_f.publish(Location(latlng_fused[0], latlng_fused[1], 0, 0))
                pub_g.publish(Location(latlng_gps[0], latlng_gps[1], 0, 0))
                # We need the last data point to calculate heading and speed
                #if self.last_x is not None:
                    #loc_msg_fused = self.make_location_message(*loc_to_lat_lng(loc_line_fused))
                    #if loc_msg_fused: pub.publish(loc_msg_fused)
                #else:
                #    self.last_x, self.last_y = global_to_local(*loc_msg_gps(loc_line_gps))
            except StopIteration:
                print("!")
            rate.sleep()

if __name__ == "__main__":
    AndroidGps().main()

#!/usr/bin/env python
import math
import subprocess
import rospy
from std_msgs.msg import Float32, String

#import matplotlib
#matplotlib.use('TkAgg')
#from matplotlib import pyplot as plt
#import matplotlib.transforms as mtransforms
#from matplotlib import animation
#from matplotlib.patches import Arrow

ADB_BINARY = "adb"
# ADB_BINARY = /home/pi/adb-1.0.31 # for the old HTC Android 6 phone
COMMAND = "%s shell dumpsys sensorservice" % ADB_BINARY
HEADER = "\nRotation Vector: last 10 events" # \n is to exclude Game Rotation Vector

# Conventions: 0 is north, ranges from -pi to pi, increasing counterclockwise
# North = 0, West = pi/2, East = -pi/2, South = +pi or -pi (numerically unstable)
def get_heading():
    output = subprocess.check_output(COMMAND, shell=True)
    try:
        idx = output.find(HEADER)
        output2 = output[idx+len(HEADER)+1:]
        idx_paren = output2.find(")")
        values_str = output2[idx_paren+1:output2.find("\n")]

        # Android gives us a quaternion :( so we convert it to a yaw angle in radians
        qx, qy, qz, qw = tuple(map(float, values_str.split(", ")[:4]))

        # thanks https://en.wikipedia.org/w/index.php?title=&oldid=884156366
        # also thanks http://marc-b-reynolds.github.io/math/2017/04/18/TaitEuler.html
        heading_arg = 2*qx*qy + 2*qz*qw
        rospy.loginfo("%f %f" % (heading_arg, 2*qw*qy + 2*qx*qz))
        if heading_arg > 1: heading_arg = 1
        elif heading_arg < -1: heading_arg = -1
        heading = math.asin(heading_arg)
        if abs(qz) > abs(qw):
            heading = math.copysign(math.pi, heading) - heading
        return heading
    except:
        return None

class AndroidCompass:
    def main(self):
        rospy.init_node("android_compass")
        pub = rospy.Publisher("/android/compass", Float32, queue_size=10)

        self.heading = 0

        # Init plotting
        #fig, ax = plt.subplots()
        #ax.set_xlim((-2,2))
        #ax.set_ylim((-2,2))
        #arrow = Arrow(0, 0, 1, 0, width=0.01)
        #ax.add_patch(arrow)
        #text = ax.text(1,1, '', fontsize=15)
        #ax.set_aspect('equal', adjustable='box')
        #plt.show(block=False)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            heading = get_heading()
            if heading is None: continue
            self.heading = heading
            pub.publish(Float32(self.heading))

            #arrow._patch_transform = mtransforms.Affine2D().rotate(self.heading)
            #text.set_text(math.degrees(self.heading))
            #plt.draw()
            #plt.pause(.001)
            rate.sleep()

if __name__ == "__main__":
    AndroidCompass().main()

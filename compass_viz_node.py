#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Float32
import Tkinter as tk

L = 80 # line length
WIDTH, HEIGHT = 200, 200
L_ORIG_X, L_ORIG_Y = WIDTH/2, HEIGHT/2

class CompassViz:
    def main(self):
        window = tk.Tk()
        window.geometry("%dx%d" % (WIDTH, HEIGHT))

        canvas = tk.Canvas(window, width=WIDTH, height=HEIGHT)

        headingvar = tk.StringVar()
        label = tk.Label(window, text = "6", anchor = tk.W)
        label.configure(width = 10, activebackground = "#33B5E5", relief = tk.FLAT)
        label_window = canvas.create_window(100, 0, anchor=tk.N, window=label)

        canvas.pack()
        line = canvas.create_line(L_ORIG_X, L_ORIG_Y, L_ORIG_X+L, L_ORIG_Y, arrow=tk.FIRST)

        def listener(data):
            heading = -data.data
            label.configure(text=("%f" % math.degrees(heading)))
            canvas.coords(line,  L_ORIG_X+L* math.cos(heading), L_ORIG_Y + L*math.sin(heading), L_ORIG_X, L_ORIG_Y)

        rospy.init_node("compass_viz_node")
        rospy.Subscriber("/android/compass", Float32, listener)

        window.mainloop()

if __name__=="__main__":
    CompassViz().main()

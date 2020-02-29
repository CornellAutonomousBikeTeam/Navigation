#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import time
import random 
import matplotlib.pyplot as plt
from std_msgs.msg import Int32MultiArray


def update_graph(data):
    d = data.data
    plot_data.append(d[0])
    

def listener():
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber("bike_state", Float32MultiArray, update_graph)
    rate = rospy.Rate(100)
    rospy.loginfo(rospy.is_shutdown())
    while not rospy.is_shutdown():
        plt.plot(plot_data)
        plt.show()
        #plt.pause(0.00001)
    
if __name__ == '__main__':
    plt.ion() # enables interactive plotting
    fig = plt.figure()
    fig.set_dpi(100) #dots per inch
    global plot_data
    plot_data = []
    ax = plt.axes(xlim=(0, 40), ylim=(-20, 20)) 
    plt.show() 
    listener()

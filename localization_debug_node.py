#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Int32, Float32, Int32MultiArray, Float32MultiArray
from msg import LocalizationDebug
from util.sensor_data import GPSData, BikeData
from util.location import global_to_local
#TODO -- Grab appropriate data and assign it in a callback
class Localization(object):
	
	def __init__(self):
		#Might want to change these to just be fields of our message
		self.msg = LocalizationDebug()
		rospy.init_node('LocalizationDebug', anonymous=True)
		
		rospy.Subscriber("gps", Float32MultiArray, gps_callback)
		rospy.Subscriber("bike_state", Float32MultiArray, bike_state_callback)
		
		self.pub = rospy.Publisher("localization_debug", LocalizationDebug, queue_size=10)
	
	def gps_callback(self,data):
		self.msg.gps_lat = data.data[0] # In degrees
        self.msg.gps_long = data.data[1]
        self.msg.gps_speed = data.data[8]
        self.msg.gps_yaw = np.deg2rad(data.data[7])
	
	def bike_state_callback(self,data):
		#TODO: Confirm these fields are correct
		self.msg.steer_encoder = data.data[4]
		self.msg.imu_yaw = data.data[11]
		
		self.msg.hall_sensor_speed = data.data[8]
		self.msg.hall_sensor_tick_counter = data.data[12]
		self.msg.hall_sensor_timestamp = data.data[13]
		
	def main_loop(self):
		rate = rospy.Rate(100)

        #Run until the nodes are shutdown (end.sh run OR start.sh was killed)
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            rate.sleep()

if __name__ == '__main__':
	Localization().main_loop()

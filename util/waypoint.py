import urllib
import json
from itertools import groupby
import sys


def get_waypoints(presets=False):
	#Gets url object and makes it into string
	# url = urllib.urlopen("http://localhost:8000/secretdatatransfer")
	if not presets:
		
		if sys.version_info[0] < 3:
			url = urllib.urlopen("https://cubike.herokuapp.com/getwaypoints")
			
		#using python 3
		else:
			url = urllib.request.urlopen("https://cubike.herokuapp.com/getwaypoints") #NEW
			
		string = url.read()

		#Make into dictionary
		dictionary = json.loads(string)
		legs = dictionary["legs"] #points list

		points = []
		points2 = []

		#Accumulates list of point tuples
		for i in legs:
			for j in (i["points"]):
				latitude = j['lat']
				longitude = j['lng']
				xy = math_convert(latitude, longitude)
				xy2 = convert2(latitude, longitude)
				points.append(xy)
				points2.append(xy2)
				
		# Remove consecutive duplicate waypoints
		points = [x[0] for x in groupby(points)]

		# Scaling Issue of points. Need to ZOOM IN because discrepancies are very small
		#print points
		# print points2
		
		return points

	else:
		point1 = math_convert(42.4450492859, -76.4836349487)
		point2 = math_convert(42.4442214966, -76.4835510254)
		return [point1, point2] 
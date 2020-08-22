#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray


#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.WhyCon_marker ={0:[],1:[],2:[]}	# Declaring dictionaries
		self.ArUco_marker = {0:[],1:[],2:[]}


		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):
		for a in self.WhyCon_marker:
			self.WhyCon_marker[a].append(float("{:.3f}".format(msg.poses[a].position.x)))
			self.WhyCon_marker[a].append(float("{:.3f}".format(msg.poses[a].position.y)))
			self.WhyCon_marker[a].append(float("{:.3f}".format(msg.poses[a].position.z)))
		print "WhyCon_marker",self.WhyCon_marker
		self.WhyCon_marker ={0:[],1:[],2:[]}


	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		for b in self.ArUco_marker:
			self.ArUco_marker[b].append(float("{:.3f}".format(msg.markers[b].pose.pose.orientation.x)))
			self.ArUco_marker[b].append(float("{:.3f}".format(msg.markers[b].pose.pose.orientation.y)))
			self.ArUco_marker[b].append(float("{:.3f}".format(msg.markers[b].pose.pose.orientation.z)))
			self.ArUco_marker[b].append(float("{:.3f}".format(msg.markers[b].pose.pose.orientation.w)))
		print "ArUco_marker",self.ArUco_marker
		print "\n"		
		self.ArUco_marker = {0:[],1:[],2:[]}

if __name__=="__main__":

	marker = Marker_detect()
	
	while not rospy.is_shutdown():
		rospy.spin()


#!/usr/bin/env python

'''

*TEAMID : 3671
*AUTHOR : SAHIL GARG
*FILENAME : data_via_rosservice.py
*THEME : HUNGRY BIRD

*FUNCTIONS : access_data()

*GLOBAL VARIABLES : self.yaw_drone,self.com

'''


from plutodrone.srv import *
from std_msgs.msg import Int64
import rospy

class request_data():

	"""docstring for request_data"""

	def __init__(self):
		self.yaw_drone = 0							#stores the yaw value of drone
		rospy.init_node('drone_board_data')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		self.com = rospy.Publisher('/yaw_value', Int64, queue_size=1)		#publish yaw value of drone to '/yaw_value'
		rospy.spin()


	'''
	*FUNCTION NAME : access_data
	*INPUT : 
	*OUTPUT : gives values of accelerometer,magnetometer,gyroscope,roll,pitch,altitude and yaw of drone
	*LOGIC : Obtaining the yaw value of drone
	*EXAMPLE CALL : access_data()
	'''
	def access_data(self, req):
		 print "accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ)
		 print "gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ)
		 print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		 print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		 self.yaw_drone = req.yaw		#obtain the yaw value of drone
		 print "altitude = " +str(req.alt)	
		 self.com.publish(self.yaw_drone)	#publishes the yaw value to "/yaw_value"
		 rospy.sleep(.1)
		 return PlutoPilotResponse(rcAUX2 =1500)

test = request_data()
	
		

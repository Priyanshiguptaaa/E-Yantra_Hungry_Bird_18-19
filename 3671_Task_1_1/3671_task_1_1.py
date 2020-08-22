#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	

'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [-8.39,4.98,27.92,0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		
		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		
		self.Kp = [9.76,5.1332,24.66,0]
		self.Ki = [0.002,0,0,0]
		self.Kd = [52.5,27.4000002,595.19999999,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------
	
		self.error=[0,0,0,0]  #to store error for [pitch, roll, throttle, yaw]
		self.prev_error = [0,0,0,0]  #to store previous error for [pitch, roll, throttle, yaw]
		self.output = [0,0,0] #to store values of pid constants [Kp,Kd,Ki] for each axis
		self.output_pitch= 0  #to store the output of pid equation for pitch axis 
		self.output_roll =0	#to store the output of pid equation for roll axis 
		self.output_throtle =0	#to store the output of pid equation for throtle axis 
		self.output_yaw = 0	#to store the output of pid equation for yaw axis 
		self.iterm_error=[0,0,0,0]	#stores sum of error during entire time of pid tuning
		
		#to limit the values of each axis

		self.max_values = [1800,1800,1800,1800]		 
		self.min_values = [1200,1200,1200,1200]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error,/zero_line
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		#------------------------Add other ROS Publishers here-----------------------------------------------------
		
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.yaw_error_pub = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.zero_line_pub = rospy.Publisher('/zero_line',Int16,queue_size=1)
		
		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, /pid_tuning_roll,/pid_tuning_yaw
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
	

		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		rospy.Subscriber('/drone_yaw', Float64, self.drone_yaw_value)		
		
		
		#------------------------------------------------------------------------------------------------------------
		
		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):
		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		
	
	# Callback function for /drone_yaw
	# This function gets executed each time when vrep publishes data on /drone_yaw
	def drone_yaw_value(self,curr_yaw):
		
		self.drone_position[3] = curr_yaw.data	
		self.temp=0
		if(self.temp==0):
			self.setpoint[3]=curr_yaw.data	#setpoint for yaw_value
			self.temp = 5
		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.6
		print(self.Ki)
		print(self.Kp)
		print(self.Kd)


	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

	# Callback function for /pid_tuning_pitch
	# This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
	def pitch_set_pid(self,alt):
		self.Kp[0] = alt.Kp*0.04
		self.Ki[0] = alt.Ki*0.001
		self.Kd[0] = alt.Kd*0.7
		print(self.Ki)
		print(self.Kp)
		print(self.Kd)


	# Callback function for /pid_tuning_roll
	# This function gets executed each time when /tune_pid publishes /pid_tuning_roll
	def roll_set_pid(self,alt):
		self.Kp[1] = alt.Kp*0.004
		self.Ki[1] = alt.Ki*0.03
		self.Kd[1] = alt.Kd*0.2
		print(self.Ki)
		print(self.Kp)
		print(self.Kd)


	# Callback function for /pid_tuning_yaw
	# This function gets executed each time when /tune_pid publishes /pid_tuning_yaw
	def yaw_set_pid(self,alt):
		self.Kp[3] = alt.Kp*0.004
		self.Ki[3] = alt.Ki*0.03
		self.Kd[3] = alt.Kd*0.2
		print(self.Ki)
		print(self.Kp)
		print(self.Kd)
 

		
	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
				
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		
		#calculating error in each axis
		for a in range(4):
			self.error[a] = self.drone_position[a] - self.setpoint[a]
			self.iterm_error[a] = self.iterm_error[a] + self.error[a]

		#output for proportional,derivative and integral controller in pitch axis
  		self.output[0] = self.Kp[0] * self.error[0]
		self.output[1] = self.Kd[0] * (self.error[0]-self.prev_error[0])
		self.output[2] = self.Ki[0] * (self.iterm_error[0])
		
		#output of pid equation in pitch axis
		self.output_pitch = sum(self.output)
		self.cmd.rcPitch = 1500 + self.output_pitch

		#limiting the output of pid equation in pitch axis
		if(self.cmd.rcPitch>self.max_values[0]):
			self.cmd.rcPitch=1800
		elif(self.cmd.rcPitch<self.min_values[0]):
			self.cmd.rcPitch=1200
		
		
		#output for proportional,derivative and integral controller in roll axis
		self.output[0] = self.Kp[1] * self.error[1]
		self.output[1] = self.Kd[1] * (self.error[1]-self.prev_error[1])
		self.output[2] = self.Ki[1] * (self.iterm_error[1])

		#output of pid equation in roll axis
		self.output_roll = sum(self.output)
		self.cmd.rcRoll = 1500 + self.output_roll


		#limiting the output of pid equation in roll axis
		if(self.cmd.rcRoll>self.max_values[1]):
			self.cmd.rcRoll=1800
		elif(self.cmd.rcRoll<self.min_values[1]):
			self.cmd.rcRoll=1200

		

		#output for proportional,derivative and integral controller in throttle axis
		self.output[0] = self.Kp[2] * self.error[2]
		self.output[1] = self.Kd[2] * (self.error[2]-self.prev_error[2])
		self.output[2] = self.Ki[2] * (self.iterm_error[2])

		#output of pid equation in throttle axis
		self.output_throtle = sum(self.output)
		self.cmd.rcThrottle = 1500 + self.output_throtle

		
		#limiting the output of pid equation in throttle axis
		if(self.cmd.rcThrottle>self.max_values[2]):
			self.cmd.rcThrottle=1800
		elif(self.cmd.rcThrottle<self.min_values[2]):
			self.cmd.rcThrottle=1200
                


		#output for proportional,derivative and integral controller in yaw axis
		self.output[0] = self.Kp[3] * self.error[3]
		self.output[1] = self.Kd[3] * (self.error[3]-self.prev_error[3])
		self.output[2] = self.Ki[3] * (self.iterm_error[3])

		#output of pid equation in yaw axis
		self.output_yaw = sum(self.output)
		self.cmd.rcYaw = 1500 + self.output_yaw


		#limiting the output of pid equation in yaw axis
		if(self.cmd.rcYaw>self.max_values[3]):
			self.cmd.rcYaw=1800
		elif(self.cmd.rcYaw<self.min_values[3]):
			self.cmd.rcYaw=1200


		#publishing error of pitch,roll,throttle,yaw on topics /pitch_error,/roll_error,/throttle_error,yaw_error
		self.pitch_error_pub.publish(self.error[0])
		self.roll_error_pub.publish(self.error[1])		
		self.alt_error_pub.publish(self.error[2])
		self.yaw_error_pub.publish(self.error[3])


		#values for previous errors
		self.prev_error[0] = self.error[0]
		self.prev_error[1] = self.error[1] 
		self.prev_error[2] = self.error[2]
		self.prev_error[3] = self.error[3]
	
	#------------------------------------------------------------------------------------------------------------------------

		#publishing data to /drone_command topic
		self.command_pub.publish(self.cmd)




if __name__ == '__main__':

	e_drone = Edrone()

	while not rospy.is_shutdown():
		e_drone.zero_line_pub.publish(0) 	#publishing "0" to /zero_line topic 
		e_drone.pid()		
		rospy.sleep(e_drone.sample_time)	#sampling time is 0.06sec

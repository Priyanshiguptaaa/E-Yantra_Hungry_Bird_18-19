#!/usr/bin/env python

'''

*TEAMID : 3671
*AUTHOR : SAHIL GARG
*FILENAME : 3671_task_4.py
*THEME : HUNGRY BIRD

*FUNCTIONS : arm(), disarm(), yaw_drone(), whycon_callback(), path_point_callback(), send_path_req(), follow_path_points(), call_pid(), through_hoop_left(), through_hoop_right(), through_hoop_forward(), through_hoop_backward(), key_callback(), pid(), path_compute_and_follow().

*GLOBAL VARIABLES : self.drone_position[], self.setpoint[],  self.Kp[], self.Ki[], self.Kd[], self.error[], self.path_points, self.output, self.whycon_detected, self.output_pitch, self.output_roll, self.output_throttle, self.output_yaw, self.iterm_error_sum_x_value, self.iterm_error_sum_y_value, self.iterm_error_sum_z_value, self.iterm_error_sum_yaw_value, self.cmd.rcRoll, self.cmd.rcPitch, self.cmd.rcThrottle, self.cmd.rcYaw,  self.current_yaw_drone, self.input_key_value, self.rate, self.command_pub, self.request_path,self.hoops.

'''

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/new_path/request		/yaw_value
						/input_key
						/vrep/waypoints

'''

# Importing the required libraries

from plutodrone.srv import *
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
		
		self.drone_position = [0.0,0.0,0.0]	# [x,y,z]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [-2,-3,22] 

		self.yaw_setpoint = 277		# desired point for yaw
		self.condition = 1		# conditional variable
							
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
		

		#initial setting of Kp, Kd and ki for [pitch, roll, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis


		self.Kp = [5.9,2.2,90]  
		self.Ki = [2.05,1.2,0]	
		self.Kd = [220,97,0]   

		self.whycon_detected = False		#determines whether whycon is detected by camera

		#--------------------------------------------------------------------------------------------------------

		self.current_yaw_drone = 0		#gives current value of drone's yaw
		self.error=[0,0,0,0]  			#to store error for [pitch, roll, throttle, yaw]
		self.prev_error = [0,0,0,0]     	#to store previous error for [pitch, roll, throttle, yaw]
		self.output = [0,0,0] 		#to store values of pid constants [Kp,Kd,Ki] for each axis
		self.output_pitch= 0  			#to store the output of pid equation for pitch axis 
		self.output_roll =0			#to store the output of pid equation for roll axis 
		self.output_throtle =0			#to store the output of pid equation for throtle axis 
		self.output_yaw = 0			#to store the output of pid equation for yaw axis 
		self.iterm_error_x=[]			#stores previous error in pitch axis
		self.iterm_error_y=[]			#stores previous error in roll axis
		self.iterm_error_z=[]			#stores previous error in throttle axis
		self.iterm_error_yaw=[]			#stores previous error in yaw axis
		self.iterm_error_sum_x_value = 0	#total sum of error in pitch axis
		self.iterm_error_sum_y_value =0		#total sum of error in roll axis
		self.iterm_error_sum_z_value =0		#total sum of error in throttle axis
		self.iterm_error_sum_yaw_value =0	#total sum of error in yaw axis
		self.input_key_value = 0		#for each key there is some value stored corresponding to key pressed
		self.path_points = []			#stores the coordinates of path computed by vrep

		self.output_yaw_kp = 0			#output of yaw for Kp
		self.output_yaw_ki = 0			#output of yaw for Ki
		self.output_yaw_kd = 0			#output of yaw for Kd
		self.yaw_Kp=30				#Kp value for yaw
		self.yaw_Ki=2.5			#Ki value for yaw
		self.yaw_Kd=0				#Kd value for yaw

		#limiting minimum and maximum values of output in pitch,roll,throttle and yaw axis 
		self.max_values = [1800,1800,1800,1800]		 
		self.min_values = [1200,1200,1200,1200]

		self.hoops = 0				#number of hoops

		# self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		# self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		
		#----------------------------------------------------------------------------------------------------------
		
		# This is the sample time in which pid runs. 
		self.rate = rospy.Rate(10)
		
		# Publishing values of pitch,roll,throttle and yaw to /drone_command
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		# Publishing value to compute path 	
		self.request_path = rospy.Publisher('/new_path/request',Int16,queue_size=1)
		
		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses to obtain whycon coordinates
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)

		# Subscribing to /yaw_value to obtain drone's current yaw value
		rospy.Subscriber('/yaw_value',Int64,self.yaw_drone)

		# Subscribing to /input_key to obtain the value of the key pressed
		rospy.Subscriber('/input_key',Int16,self.key_callback)
		
		# Subscribing to /vrep/waypoints to obtain the coordinates of path computed by vrep
		rospy.Subscriber('/vrep/waypoints',PoseArray,self.path_point_callback)
		
		#------------------------------------------------------------------------------------------------------------

	'''
	*FUNCTION NAME : disarm
	*INPUT : 
	*OUTPUT : publishes the value of 1100 to self.cmd.rcAUX4 
	*LOGIC :Disarming condition of the drone
	*EXAMPLE CALL :disarm()
	'''
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	'''
	*FUNCTION NAME :arm
	*INPUT :
	*OUTPUT : publishes the value of 1500 to each parameter of drone
	*LOGIC :Arming condition of the drone
	*EXAMPLE CALL :arm()
	'''
	def arm(self):
		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
		
		
	'''
	*FUNCTION NAME :yaw_drone
	*INPUT : 
	*OUTPUT : store this value in self.current_yaw_drone
	*LOGIC :callback function to obtain the yaw value of drone
	*EXAMPLE CALL :yaw_drone()
	'''
	def yaw_drone(self,req):
		self.current_yaw_drone = req.data
	
	
	'''
	*FUNCTION NAME :whycon_callback
	*INPUT : coordinates of whycon marker
	*OUTPUT : store the coordinates in self.drone_position list
	*LOGIC :The function gets executed each time when /whycon node publishes /whycon/poses
	*EXAMPLE CALL :whycon_callback()
	'''
	# Whycon callback function
	def whycon_callback(self,msg):
		self.start_time = time.time()
		self.drone_position[0] = msg.poses[0].position.y
		self.drone_position[1] = msg.poses[0].position.x
		self.drone_position[2] = msg.poses[0].position.z
		self.whycon_detected=True

		#---------------------------------------------------------------------------------------------------------------


	'''
	*FUNCTION NAME :path_point_callback
	*INPUT : coordinates of the path computed by vrep
	*OUTPUT : store these coordinates in self.path_points list
	*LOGIC :function to store the recieved path points or coordinates of path computed in list (self.path_points)
	*EXAMPLE CALL :path_point_callback()
	'''
	def path_point_callback(self,msg):
		for a in range(len(msg.poses)):
			self.path_points.append([msg.poses[a].position.y,msg.poses[a].position.x,msg.poses[a].position.z])
		print(self.path_points)


	'''
	*FUNCTION NAME :send_path_req
	*INPUT : number that must be send to vrep to compute the corresponding path
	*OUTPUT :publishes the input number to topic (/new_path/request)
	*LOGIC :function to send request to vrep to compute path
	*EXAMPLE CALL :send_path_req()
	'''
	def send_path_req(self,num):
		self.request_path.publish(num) 		#publishes data to '/new_path/request' topic.It pulishes the path number


	'''
	*FUNCTION NAME :follow_path_points
	*INPUT :
	*OUTPUT :set the setpoint for drone according to the coordinates of the path computed
	*LOGIC :function to follow the coordinates of the path computed by vrep
	*EXAMPLE CALL :follow_path_points()
	'''
	def follow_path_points(self):
		
		# set the next setpoint for drone 
		for x in range(0,len(self.path_points),5):
			
		        self.setpoint = self.path_points[x]
			self.call_pid()
		self.path_points = []

		
	'''
	*FUNCTION NAME :call_pid
	*INPUT :
	*OUTPUT : run pid function
	*LOGIC :run pid till the drone stabilizes with a error of '1' in each axis
	*EXAMPLE CALL :call_pid()
	'''
	def call_pid(self):

		self.pid()
		while(abs(self.error[0])>1.5 or abs(self.error[1])>1.5 or abs(self.error[2])>1):	
			self.pid()
			self.rate.sleep()
	

	'''	
	*FUNCTION NAME :through_hoop_forward
	*INPUT :
	*OUTPUT : set the pitch value to 1410 and roll to 1500 for 0.62ms to move the drone in forward direction so that drone can pass through the hoop
	*LOGIC :Function to pass the drone through hoop when it reaches to hoop's back side
	*EXAMPLE CALL :through_hoop_forward()
	'''	
	def through_hoop_forward(self):
		
		self.cmd.rcPitch = 1410
		self.cmd.rcRoll = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.62)


	'''	
	*FUNCTION NAME :through_hoop_left
	*INPUT :
	*OUTPUT : set the pitch value to 1460 and roll to 1430 for 0.70ms to move the drone in left direction so that drone can pass through the hoop
	*LOGIC :Function to pass the drone through hoop when it reaches to hoop's front side
	*EXAMPLE CALL :through_hoop_left()
	'''	
	def through_hoop_left(self):
		
		self.cmd.rcPitch = 1460
		self.cmd.rcRoll = 1430
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.70)

	'''	
	*FUNCTION NAME :through_hoop_backward
	*INPUT :
	*OUTPUT : set the pitch value to 1450 and roll to 1520 for 0.45ms to move the drone in backward direction so that drone can pass through the hoop
	*LOGIC :Function to pass the drone through hoop when it reaches to hoop's front side
	*EXAMPLE CALL :through_hoop_backward()
	'''	
	def through_hoop_backward(self):
		
		self.cmd.rcPitch = 1450
		self.cmd.rcRoll = 1520
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.45)
		
	'''	
	*FUNCTION NAME :through_hoop_right
	*INPUT :
	*OUTPUT : set the pitch value to 1470 and roll to 1560 for 0.47ms to move the drone in right direction so that drone can pass through the hoop
	*LOGIC :Function to pass the drone through hoop when it reaches to hoop's back side
	*EXAMPLE CALL :through_hoop_right()
	'''	
	def through_hoop_right(self):
		
		self.cmd.rcPitch = 1470
		self.cmd.rcRoll = 1560
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.47)
		
	
	'''
	*FUNCTION NAME :key_callback
	*INPUT : value of key pressed
	*OUTPUT : store the input value to self.input_key_value
	*LOGIC :callback function to recieve the value of key pressed
	*EXAMPLE CALL :key_callback()
	'''
	def key_callback(self,msg):	

		self.input_key_value = msg.data
		print(self.input_key_value)

		# set the number of hoops
		if(self.input_key_value==15):
			self.hoops=1
		elif(self.input_key_value==30):
			self.hoops=2
		elif(self.input_key_value==40):
			self.hoops=3
	#----------------------------------------------------------------------------------------------------------------------


	'''
	*FUNCTION NAME :pid
	*INPUT :
	*OUTPUT : apply pid to stable the drone
	*LOGIC : calculate the error in position of drone then multiply this error with corresponding pid parameters and obtain the value of rcPitch,rcRoll,rcThrottle and rcYaw then publish those values.
	*EXAMPLE CALL :pid()
	'''
	def pid(self):
				
	#-----------------------------------------------------PID algorithm here--------------------------------------------------------------
		

		#calculating error in each axis
		for a in range(3):
			self.error[a] = self.drone_position[a] - self.setpoint[a]

		self.error[3] = self.current_yaw_drone -  self.yaw_setpoint
 		
		#execute this condition when whycon is not detected
		if (self.whycon_detected==False):
			self.cmd.rcPitch = 1500
			self.cmd.rcRoll = 1500
			self.cmd.rcThrottle = self.cmd.rcThrottle
			self.command_pub.publish(self.cmd)
			return

		# store the error value in x
		self.iterm_error_x.append(self.error[0])

		# when the length of the above list becomes 3, delete the first element(oldest error)
		if len(self.iterm_error_x) > 3:
			self.iterm_error_x.pop(0)
		# total sum of error in pitch axis
		self.iterm_error_sum_x_value = sum(self.iterm_error_x)

		# store the error value in y
		self.iterm_error_y.append(self.error[1])
		
		# when the length of the above list becomes 3, delete the first element(oldest error)
		if len(self.iterm_error_y) > 3:
			self.iterm_error_y.pop(0)
		# total sum of error in roll axis
		self.iterm_error_sum_y_value = sum(self.iterm_error_y)

		# store the error value in z
		self.iterm_error_z.append(self.error[2])

		# when the length of the above list becomes 3, delete the first element(oldest error)
		if len(self.iterm_error_z) > 3:
			self.iterm_error_z.pop(0)
		# total sum of error in throttle axis
		self.iterm_error_sum_z_value = sum(self.iterm_error_z)

		# store the error value in yaw
		self.iterm_error_yaw.append(self.error[3])

		# when the length of the above list becomes 3, delete the first element(oldest error)
		if len(self.iterm_error_yaw) > 3:
			self.iterm_error_yaw.pop(0)
		# total sum of error in yaw axis
		self.iterm_error_sum_yaw_value = sum(self.iterm_error_yaw)
		

		#output for proportional,derivative and integral controller in pitch axis
  		self.output[0] = self.Kp[0] * self.error[0]
		self.output[1] = self.Kd[0] * (self.error[0]-self.prev_error[0])
		self.output[2] = self.Ki[0] * (self.iterm_error_sum_x_value)
		
		#output of pid equation in pitch axis
		self.output_pitch = sum(self.output)
		self.cmd.rcPitch = 1500 + (self.output_pitch)

		#limiting the output of pid equation in pitch axis
		if(self.cmd.rcPitch>self.max_values[0]):
			self.cmd.rcPitch=self.max_values[0]
		elif(self.cmd.rcPitch<self.min_values[0]):
			self.cmd.rcPitch=self.min_values[0]
		

		#output for proportional,derivative and integral controller in roll axis
		self.output[0] = self.Kp[1] * self.error[1]
		self.output[1] = self.Kd[1] * (self.error[1]-self.prev_error[1])
		self.output[2] = self.Ki[1] * (self.iterm_error_sum_y_value)
		
		#output of pid equation in roll axis
		self.output_roll = sum(self.output)
		self.cmd.rcRoll = 1500 - (self.output_roll)
		
		#limiting the output of pid equation in roll axis
		if(self.cmd.rcRoll>self.max_values[1]):
			self.cmd.rcRoll=self.max_values[1]
		elif(self.cmd.rcRoll<self.min_values[1]):
			self.cmd.rcRoll=self.min_values[1]

		
		#output for proportional,derivative and integral controller in throttle axis
		self.output[0] = self.Kp[2] * self.error[2]
		self.output[1] = self.Kd[2] * (self.error[2]-self.prev_error[2])
		self.output[2] = self.Ki[2] * (self.iterm_error_sum_z_value)
		
		#output of pid equation in throttle axis
		self.output_throtle = sum(self.output)
		self.cmd.rcThrottle = 1500 + (self.output_throtle)
		
		#limiting the output of pid equation in throttle axis
		if(self.cmd.rcThrottle>self.max_values[2]):
			self.cmd.rcThrottle=self.max_values[2]
		elif(self.cmd.rcThrottle<self.min_values[2]):
			self.cmd.rcThrottle=self.min_values[2]


		#output for proportional,derivative and integral controller in yaw axis
		self.output[0] = self.yaw_Kp * self.error[3]
		self.output[1] = self.yaw_Kd * (self.error[3]-self.prev_error[3])
       		self.output[2] = self.yaw_Ki * self.iterm_error_sum_yaw_value
	
		#output of pid equation in yaw axis
		self.output_yaw = sum(self.output)
		self.cmd.rcYaw = 1500 - self.output_yaw
		
		#limiting the output of pid equation in yaw axis
		if(self.cmd.rcYaw>self.max_values[3]):
			self.cmd.rcYaw=self.max_values[3]
		elif(self.cmd.rcYaw<self.min_values[3]):
			self.cmd.rcYaw=self.min_values[3]


		#publishing data to /drone_command topic
		self.command_pub.publish(self.cmd)


		#values for previous errors
		self.prev_error[0] = self.error[0]
		self.prev_error[1] = self.error[1] 
		self.prev_error[2] = self.error[2]
		self.prev_error[3] = self.yaw_error_value

		self.whycon_detected=False	


	'''
	*FUNCTION NAME :
	*INPUT :
	*OUTPUT : follow the path from start_point to back_of_hoop and then from front_of_hoop to start_point
	*LOGIC : limits the maximum and minimum values of pitch,roll and throttle to control speed of drone and follow the path gently then pass through the hoop then drone follows the path from front_of_hoop to start_point
	*EXAMPLE CALL : path_compute_and_follow()
	'''
	def path_compute_and_follow(self):

		
		# checks the error is less than 0.7 or not in each axis  
		if(abs(self.error[0])<1 and abs(self.error[1])<1 and abs(self.error[2])<1):
			
			# change the maximum and minimum values to control the speed of drone
			self.max_values = [1530,1530,1800]		 
			self.min_values = [1470,1470,1200]

			# checks for the no. of hoops and accordingly traverse through arena
 
			if(self.hoops==1):

			
				if(self.condition==1):
					self.send_path_req(1)	      #send request to compute path 
					self.condition=2

				elif(len(self.path_points)>0 and self.condition==2):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=3

				elif(self.condition==3):
					
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_backward()  #pass the drone through the hoop
					self.condition =4				
					self.send_path_req(3)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==4):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=5
					self.disarm()		      # DISARMING THE DRONE


			if(self.hoops==2):
								
				if(self.condition==1):
					self.send_path_req(1)	       #send request to compute path 
					self.condition=2

				elif(len(self.path_points)>0 and self.condition==2):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=3

				elif(self.condition==3):
					
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_left()      #pass the drone through the hoop
					self.condition =4				
					self.send_path_req(2)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==4):
							
					self.follow_path_points()     #calls the function to follow the path
					self.condition=5
					
				elif(self.condition==5):
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_backward()  #pass the drone through the hoop
					self.condition =6				
					self.send_path_req(3)	      #send request to compute path 
				elif(len(self.path_points)>0 and self.condition==6):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=7

				elif(self.condition==7):
					
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_right()     #pass the drone through the hoop
					self.condition = 8				
					self.send_path_req(4)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==8):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=9
					self.disarm()		      # DISARMING THE DRONE


			if(self.hoops==3):
								
				if(self.condition==1):
					self.send_path_req(1)	       #send request to compute path 
					self.condition=2

				elif(len(self.path_points)>0 and self.condition==2):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=3

				elif(self.condition==3):
					
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_left()      #pass the drone through the hoop
					self.condition =4				
					self.send_path_req(2)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==4):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=5
					
				elif(self.condition==5):
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_right()     #pass the drone through the hoop
					self.condition =6				
					self.send_path_req(3)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==6):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=7

				elif(self.condition==7):
					
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_left()      #pass the drone through the hoop
					self.condition = 8				
					self.send_path_req(4)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==8):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=9

				elif(self.condition==9):
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_left()      #pass the drone through the hoop
					self.condition =10				
					self.send_path_req(5)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==10):
							
					self.follow_path_points()     #calls the function to follow the path computed 
					self.condition=11

				elif(self.condition==11):
					
					self.call_pid()		      #stabilize the drone upto predefined error
					self.through_hoop_right_()    #pass the drone through the hoop
					self.condition = 12				
					self.send_path_req(6)	      #send request to compute path 
			
				elif(len(self.path_points)>0 and self.condition==12):
							
					self.follow_path_points()     #calls the function to follow the path 
					self.condition=13
					self.disarm()		      # DISARMING THE DRONE


	#------------------------------------------------------------------------------------------------------------------------


if __name__ == '__main__':
	
	e_drone = Edrone()
	
	# run this loop until rospy is not interrupted
	while not rospy.is_shutdown():
		
		if(e_drone.input_key_value == 70):
			e_drone.disarm()				# DISARMING THE DRONE
			e_drone.arm()					# ARMING THE DRONE
			e_drone.input_key_value =1			# reset the value of 'self.input_key_value'

		elif(e_drone.input_key_value ==1):
			e_drone.pid()					# call pid function to stablizethe drone
			e_drone.path_compute_and_follow()		# send request to compute the path and follow it  	
			e_drone.rate.sleep()				# sampling time is 10Hz

	e_drone.disarm()		# DISARMING THE DRONE


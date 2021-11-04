'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 1C of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''

# Team ID:			[ 1707 ]
# Author List:		[ Parth Shah, Shubhankar Riswadkar, Chirag Jain, Bhavya Vira ]
# Filename:			task_1c.py
# Functions:		read_distance_sensor, control_logic
# 					[ Comma separated list of functions in this file ]
# Global variables:	
# 					[ List of global variables defined in this file ]



####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
##############################################################
import  sys
import traceback
import time
import os
import math

try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()
	
try:
	import task_1b

except ImportError:
	print('\n[ERROR] task_1b.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure task_1b.py is present in this current directory.\n')
	sys.exit()
		
except Exception as e:
	print('Your task_1b.py throwed an Exception, kindly debug your code!\n')
	traceback.print_exc(file=sys.stdout)
	sys.exit()

##############################################################

def read_distance_sensor(client_id, sensor_handle):
	"""
	Purpose:
	---
	This function returns the distance from a proximity sensor

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	`sensor_handle`  : [ integer ]
	    handle of the proximity sensor

	Returns:
	---
	`detected`   :  [ boolean ]
	    returns True if proximity sensor detects wall and False if no wall is detected.
	
	`distance`   :  [ float ]
	    returns the closest distance from wall if wall is detected, returns -1 if no wall is detected.

	Example call:
	---
	detected, distance = read_distance_sensor(client_id, sensor_handle)
	"""
	distance = -1
	detected = False

	##############	ADD YOUR CODE HERE	##############
	sensor_reading = sim.simxReadProximitySensor(client_id, sensor_handle, sim.simx_opmode_buffer)
	detected = sensor_reading[1]
	if detected:
		distance = sensor_reading[2][2]
	else:
		distance = -1
	##################################################
	return detected, distance


def control_logic(client_id):
	"""
	Purpose:
	---
	This function should implement the control logic for the given problem statement
	You are required to actuate the rotary joints of the robot in this function, such that
	it traverses the points in given order

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	Returns:
	---
	None

	Example call:
	---
	control_logic(client_id)
	"""

	##############  ADD YOUR CODE HERE  ##############
	#Initializing Wheel Handles
	_, left_wheel_handle = sim.simxGetObjectHandle(client_id, 'left_joint', sim.simx_opmode_blocking)
	_, right_wheel_handle = sim.simxGetObjectHandle(client_id, 'right_joint', sim.simx_opmode_blocking)
	#The bot is stopped - Set initial velocity to 0
	_ = sim.simxSetJointTargetVelocity(client_id, left_wheel_handle, 0, sim.simx_opmode_streaming)
	_ = sim.simxSetJointTargetVelocity(client_id, right_wheel_handle, 0, sim.simx_opmode_streaming)


	#Intitializing Proximity Sensor Handles
	_, distance_sensor_1_handle = sim.simxGetObjectHandle(client_id, 'distance_sensor_1', sim.simx_opmode_blocking)
	_, distance_sensor_2_handle = sim.simxGetObjectHandle(client_id, 'distance_sensor_2', sim.simx_opmode_blocking)
	#Start streaming Proximity Sensor Data to buffer
	sim.simxReadProximitySensor(client_id, distance_sensor_1_handle, sim.simx_opmode_streaming)
	sim.simxReadProximitySensor(client_id, distance_sensor_2_handle, sim.simx_opmode_streaming)
	
	#Read Proximity Sensor Data from buffer
	detected_1, distance_1 = read_distance_sensor(client_id, distance_sensor_1_handle)
	detected_2, distance_2 = read_distance_sensor(client_id, distance_sensor_2_handle)


	#Starting the bot
	mode = "forward"
	_ = sim.simxSetJointTargetVelocity(client_id, left_wheel_handle, 1, sim.simx_opmode_streaming)
	_ = sim.simxSetJointTargetVelocity(client_id, right_wheel_handle,1, sim.simx_opmode_streaming)

	turns = 0	#Number of turns taken
	prev_distance_2 = distance_2
	while(1):
		detected_1, distance_1 = read_distance_sensor(client_id, distance_sensor_1_handle)
		detected_2, distance_2 = read_distance_sensor(client_id, distance_sensor_2_handle)

		# Corner - Entry Condition
		if (mode == "forward") and (detected_1) and (distance_1 <= distance_2):
			mode = "turning"
			#The bot reaches A after completing 3 turns and going forward, stop the bot before taking the 4th turn
			if turns == 3:
				break
			#Rotate the bot
			_ = sim.simxSetJointTargetVelocity(client_id, left_wheel_handle, -0.15, sim.simx_opmode_streaming)
			_ = sim.simxSetJointTargetVelocity(client_id, right_wheel_handle, 0.15, sim.simx_opmode_streaming)
			turns += 1

		# Corner - Exit Condition
		if (mode == "turning") and (not detected_1) and (distance_2 > prev_distance_2):
			mode = "forward"
			#Move Forward Bot
			_ = sim.simxSetJointTargetVelocity(client_id, left_wheel_handle, 1, sim.simx_opmode_streaming)
			_ = sim.simxSetJointTargetVelocity(client_id, right_wheel_handle,1, sim.simx_opmode_streaming)
		
		prev_distance_2 = distance_2

	#Stop the bot
	_ = sim.simxSetJointTargetVelocity(client_id, left_wheel_handle, 0, sim.simx_opmode_oneshot)
	_ = sim.simxSetJointTargetVelocity(client_id, right_wheel_handle, 0, sim.simx_opmode_oneshot)


	##################################################

	## You are NOT allowed to make any changes in the code below ##
if __name__ == "__main__":

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = task_1b.init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Starting the Simulation
			try:
				return_code = task_1b.start_simulation(client_id)

				if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
					print('\nSimulation started correctly in CoppeliaSim.')

				else:
					print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
					print('start_simulation function is not configured correctly, check the code!')
					print()
					sys.exit()

			except Exception:
				print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
				print('Stop the CoppeliaSim simulation manually.\n')
				traceback.print_exc(file=sys.stdout)
				print()
				sys.exit()

		else:
			print('\n[ERROR] Failed connecting to Remote API server!')
			print('[WARNING] Make sure the CoppeliaSim software is running and')
			print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
			print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

	try:
		control_logic(client_id)
		time.sleep(1)        

		try:
			return_code = task_1b.stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
				try:
					task_1b.exit_remote_api_server(client_id)
					if (task_1b.start_simulation(client_id) == sim.simx_return_initialize_error_flag):
						print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

					else:
						print('\n[ERROR] Failed disconnecting from Remote API server!')
						print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

				except Exception:
					print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
					print('Stop the CoppeliaSim simulation manually.\n')
					traceback.print_exc(file=sys.stdout)
					print()
					sys.exit()
									  
			else:
				print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
				print('[ERROR] stop_simulation function is not configured correctly, check the code!')
				print('Stop the CoppeliaSim simulation manually.')
		  
			print()
			sys.exit()

		except Exception:
			print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

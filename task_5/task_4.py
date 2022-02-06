'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
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
# Author List:		[ Parth Shah, Chirag Jain, Shubhankar Riswadkar, Bhavya Vira ]
# Filename:			task_4.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

from pydoc import cli
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
from pyzbar.pyzbar import decode

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
import task_1b
import task_2a
from task_3 import *

def wait_for_gripper_completion(client_id, command):
	emptybuff = bytearray()
	return_code, status, _, _, _ = sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'check_status',[],[],[],emptybuff,sim.simx_opmode_blocking)
	print(status)
	succes_code = 0
	if command == ["open"]:
		succes_code = 1
	elif command == ["close"]:
		succes_code = 2

	while(status[0] != succes_code):
		return_code, status, _, _, _ = sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'check_status',[],[],[],emptybuff,sim.simx_opmode_blocking)	

def wait_for_ik_completion(client_id):
	emptybuff = bytearray()
	return_code, still_error, _, _, _ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'check_ik',[],[],[],emptybuff,sim.simx_opmode_blocking)
	while(still_error[0]):
		return_code, still_error, _, _, _ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'check_ik',[],[],[],emptybuff,sim.simx_opmode_blocking)

def wait_for_fk_completion(client_id):
	emptybuff = bytearray()
	return_code, still_error, _, _, _ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'check_fk',[],[],[],emptybuff,sim.simx_opmode_blocking)
	while(still_error[0]):
		return_code, still_error, _, _, _ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'check_fk',[],[],[],emptybuff,sim.simx_opmode_blocking)

def open_gripper(client_id):
	command = ["open"]
	emptybuff = bytearray()
    
	return_code = 1
	while return_code != 0:
		return_code, _, _, _, _ = sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)
	wait_for_gripper_completion(client_id, command)
	

def close_gripper(client_id):
	command = ["close"]
	emptybuff = bytearray()
    
	return_code = 1
	while return_code != 0:
		return_code, _, _, _, _ = sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)
    
	wait_for_gripper_completion(client_id, command)


def arm_move_to_target(client_id, reference_frame, x, y, z):
	command = [reference_frame, x, y, z]
	emptybuff = bytearray()
	return_code, _, _, _, _ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'goal_IK',[],command,[],emptybuff,sim.simx_opmode_blocking)
	wait_for_ik_completion(client_id)

def arm_go_to_start_pose(client_id):
	emptybuff = bytearray()
	return_code, _, _, _, _ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'home_FK',[],[],[],emptybuff,sim.simx_opmode_blocking)
	wait_for_fk_completion(client_id)

def arm_go_to_rack(client_id, reference_frame):
	rack_x, rack_y, rack_z = (-0.5, 0, -0.2)	
	arm_move_to_target(client_id, reference_frame, rack_x, rack_y, rack_z)
	
def berry_detection_wrapper(client_id):
	return_code, berry_detector_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, berry_detector_handle)
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, berry_detector_handle)
	
	if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
		transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
		transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)

		if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):
			berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
			berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
			labelled_image = task_2a.get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)
			
			# cv2.imshow('transformed image', transformed_image)
			# cv2.imshow('transformed depth image', transformed_depth_image)
			# cv2.imshow('labelled image', labelled_image)

	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	berry_positions_dictionary['Strawberry'] = sorted(berry_positions_dictionary['Strawberry'], key=calculate_r)
	berry_positions_dictionary['Lemon'] = sorted(berry_positions_dictionary['Lemon'], key=calculate_r)
	berry_positions_dictionary['Blueberry'] = sorted(berry_positions_dictionary['Blueberry'], key=calculate_r)

	return berry_positions_dictionary, berry_detector_handle

def calculate_r(position):
	r = math.sqrt((position[0] ** 2) + (position[1] ** 2) + (position[2] ** 2))
	return r

############################# EVAL FUNCTION #############################
def send_identified_berry_data(client_id,berry_name,x_coor,y_coor,depth):
	"""
	Purpose:
	---
	Teams should call this function as soon as they identify a berry to pluck. This function should be called only when running via executable.
	
	NOTE: 
	1. 	Correct Pluck marks will only be awarded if the team plucks the last detected berry. 
		Hence before plucking, the correct berry should be identified and sent via this function.

	2.	Accuracy of detection should be +-0.025m.

	Input Arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable

	'berry_name'		:	[ string ]
			name of the detected berry.

	'x_coor'			:	[ float ]
			x-coordinate of the centroid of the detected berry.

	'y_coor'			:	[ float ]
			y-coordinate of the centroid of the detected berry.

	'depth'			:	[ float ]
			z-coordinate of the centroid of the detected berry.

	Returns:
	---
	`return_code`		:	[ integer ]
			A remote API function return code
			https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes

	Example call:
	---
	return_code=send_identified_berry_data(berry_name,x_coor,y_coor)
	
	"""
	##################################################
	## You are NOT allowed to make any changes in the code below. ##
	emptybuff = bytearray()

	if(type(berry_name)!=str):
		berry_name=str(berry_name)

	if(type(x_coor)!=float):
		x_coor=float(x_coor)

	if(type(y_coor)!=float):
		y_coor=float(y_coor)	
	
	if(type(depth)!=float):
		depth=float(depth)
	
	data_to_send=[berry_name,str(x_coor),str(y_coor),str(depth)]					
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'eval_bm',sim.sim_scripttype_childscript,'detected_berry_by_team',[],[],data_to_send,emptybuff,sim.simx_opmode_blocking)
	return return_code
	
	##################################################

def pluck_and_deposit(client_id, reference_frame, coordinates):
	berry_x, berry_y, berry_z = coordinates

	open_gripper(client_id)
	time.sleep(0.5)
	
	arm_move_to_target(client_id, reference_frame, berry_x, berry_y, round(berry_z, 6))
	time.sleep(1)

	close_gripper(client_id)
	time.sleep(0.5)

	berry_type = 'Blueberry'
	if berry_type == 'Blueberry':
		arm_move_to_target(client_id, reference_frame, berry_x, berry_y, 0)
		# time.sleep(1)
		arm_go_to_start_pose(client_id)
		# time.sleep(1)
	elif berry_type == 'Lemon':
		arm_go_to_start_pose(client_id)
		# time.sleep(1)
	else:
		arm_go_to_rack(client_id, reference_frame)
		# time.sleep(0.7)

	arm_go_to_rack(client_id, reference_frame)
	# time.sleep(0.5)
	
	open_gripper(client_id)
	# 	time.sleep(0.5)
	
	return_code = arm_go_to_start_pose(client_id)
	time.sleep(0.5)


##############################################################


def task_4_primary(client_id):
	"""
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot traverses to the vertical rack, detects, plucks & deposits a berry of each color.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	
	Returns:
	---
	
	Example call:
	---
	task_4_primary(client_id)
	
	"""
	wheel_joints = init_setup(client_id)
	set_bot_movement(client_id, wheel_joints, 0, 0, 0)

	# arm_go_to_rack(client_id, None)
	# return 

	target_points = [(4, 3)]
	task_3_primary(client_id, target_points)
	
	_, start_y_enc, _ = wrapper_encoders(client_id)
	while(1):
		set_bot_movement(client_id, wheel_joints, 5, 0, 0)
		x_enc, y_enc, rot_enc = wrapper_encoders(client_id)
		if y_enc - start_y_enc > 0.08:
			break
	set_bot_movement(client_id, wheel_joints, 0, 0, 0)

	berry_positions_dictionary, berry_detector_handle = berry_detection_wrapper(client_id)
	berry_index = 0
	for berry_type in berry_positions_dictionary.keys():
		berry_x, berry_y, berry_z = berry_positions_dictionary[berry_type][berry_index]
		
		send_identified_berry_data(client_id, berry_type, berry_x, berry_y, berry_z)

		open_gripper(client_id)
		time.sleep(0.5)
		
		arm_move_to_target(client_id, berry_detector_handle, berry_x, berry_y, round(berry_z, 6))
		time.sleep(1)

		close_gripper(client_id)
		time.sleep(0.5)

		if berry_type == 'Blueberry':
			return_code = arm_move_to_target(client_id, berry_detector_handle, berry_x, berry_y, 0)
			time.sleep(1)
			return_code = arm_go_to_start_pose(client_id)
			time.sleep(1)
		elif berry_type == 'Lemon':
			return_code = arm_go_to_start_pose(client_id)
			time.sleep(1)
		else:
			return_code = arm_go_to_rack(client_id, berry_detector_handle)
			time.sleep(0.7)

		return_code = arm_go_to_rack(client_id, berry_detector_handle)
		time.sleep(0.5)
		
		open_gripper(client_id)
		time.sleep(0.5)
		
		return_code = arm_go_to_start_pose(client_id)
		time.sleep(0.5)

if __name__ == "__main__":


	##################################################
	## You are NOT allowed to make any changes in the code below ##

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Starting the Simulation
			try:
				return_code = start_simulation(client_id)

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

		task_4_primary(client_id)
		time.sleep(1)        

		try:
			return_code = stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
				try:
					exit_remote_api_server(client_id)
					if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
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
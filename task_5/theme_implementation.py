'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement the Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ BM_1707 ]
# Author List:		[ Parth Shah, Chirag Jain, Shubhankar Riswadkar, Bhavya Vira ]
# Filename:			theme_implementation.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import re
from anyio import sleep
import cv2
from cv2 import rotate
import numpy as np
import os, sys
import traceback
import math
import time
import sys
import json
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
from task_4 import *

bm_map = None
requirements = {}

def rotate_robot(client_id, wheel_joints, angle):
	_, start_y_enc, rot_enc = wrapper_encoders(client_id)
	rot_enc = rot_enc * 180 / 0.5
	start_rot = rot_enc
	
	err = 0
	prev_err = 0
	cum_err = 0

	P, I, D = (0, 0, 0)
	delta_t = 0.01
	k_p = -0.02
	k_i = 0
	k_d = 0
	while(1):
		x_enc, y_enc, rot_enc = wrapper_encoders(client_id)
		# rot_enc = rot_enc * 180 / 0.5
		total_rot = rot_enc - start_rot

		err = total_rot - angle	
		cum_err += err
		P = (err * k_p)
		D = (((err - prev_err) * k_d) / delta_t)
		I = (k_i * cum_err)
		rotation_cmd = P + I + D

		print(P, I, D)
		set_bot_movement(client_id, wheel_joints, 0, 0, rotation_cmd)
	
		if abs(total_rot) > 0.25:
			break
		prev_err = err
		# sleep(delta_t)

def go_to_rack(client_id, rack_id):
	target_points = None
	if rack_id == 1:
		target_points = [(1, 10)]
	elif rack_id == 2:
		target_points = [(7, 10)]

	current_position = get_current_position(client_id)
	current_position = tuple(map(int, current_position.replace('(','').replace(')','').split(',')))

	task_3_primary(client_id, current_position, target_points, bm_map)

def read_config(filename):
	config_file = open(filename)
	theme_config = json.load(config_file)
	print(theme_config)
	
	theme_requirements = {}
	for key in list(theme_config.keys()):
		req = theme_config[key]
		req = req.split("_")
		
		print(key)
		berry_type = None
		if key == "B":
			berry_type = "Blueberry"
		elif key == "L":
			berry_type = "Lemon"
		else:
			berry_type = "Strawberry"
		theme_requirements[berry_type] = {}
		theme_requirements[berry_type]["number_of_berries"] = int(req[0])
		theme_requirements[berry_type]["deposit_box"] = int(req[1][-1])
	return theme_requirements

def requirement_satisfied(berry_type):
	satisfied = False
	if requirements[berry_type]["number_of_berries"] == 0:
		satisfied = True
	return satisfied

def plucking_completed():
	completed = True 
	for berry_type in list(requirements.keys()):
		completed &= requirement_satisfied(berry_type)
	return completed

def pluck_from_zone(client_id, zone_id):
	target_points = None
	if zone_id == 1:
		target_points = [(7, 1)]
		rotate_angle = 0
	elif zone_id == 2:
		target_points = [(7, 7)]
		rotate_angle = 90
	elif zone_id == 3:
		target_points = [(1, 7)]
		rotate_angle = 180
	elif zone_id == 4:
		target_points = [(1, 1)]
		rotate_angle = -90
	
	current_position = get_current_position(client_id)

	task_3_primary(client_id, current_position, target_points, bm_map)

	# rotate required
	
	#do
		# capture berry data
		# pick berry to be plucked
		# pluck 
		# deposit
	# while req not satisfied or no more necessary berries

	# rotate reverse

	global requirements
	for berry_type in list(requirements.keys()):
		if requirement_satisfied(berry_type):
			continue
		plucked = 0
		
		num_of_berries = requirements[berry_type]["number_of_berries"]
		for i in range(num_of_berries):
			berry_positions_dictionary, berry_detector_handle = berry_detection_wrapper(client_id)

			if len(berry_positions_dictionary[berry_type]) == 0:
				break

			coordinates = berry_positions_dictionary[berry_type][0]
			pluck_and_deposit(client_id, berry_detector_handle, coordinates)
			plucked += 1

		requirements[berry_type]["number_of_berries"] -= plucked
		


	berry_positions_dictionary, berry_detector_handle = berry_detection_wrapper(client_id)
	


##############################################################

def theme_implementation_primary(client_id, rooms_entry):
	"""
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot completes the Theme Implementation.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	`rooms_entry`         :   [ list of tuples ]
		Room entry co-ordinate of each room in order.

	
	Returns:
	---
	
	Example call:
	---
	theme_implementation_primary(client_id, rooms_entry)
	
	"""
	wheel_joints = init_setup(client_id)
	set_bot_movement(client_id, wheel_joints, 0, 0, 0)
	
	global bm_map 
	bm_map = generate_map(rooms_entry)

	global requirements
	requirements = read_config("Theme_Config.json")
	print(requirements)

	# pluck_from_zone(client_id, 1)
	# pluck_from_zone(client_id, 2)
	pluck_from_zone(client_id, 3)
	# pluck_from_zone(client_id, 4)
	
	# go_to_rack(client_id, 1)
	# go_to_rack(client_id, 2)

	# time.sleep(3)
	# _, start_y_enc, rot_enc = wrapper_encoders(client_id)
	# start_rot = rot_enc
	# set_bot_movement(client_id, wheel_joints, 0, 0, -3)
	# while(1):
	# 	x_enc, y_enc, rot_enc = wrapper_encoders(client_id)
	# 	total_rot = rot_enc - start_rot
	# 	if abs(total_rot) > 0.25:
	# 		break
	
	# _, start_y_enc, rot_enc = wrapper_encoders(client_id)
	# start_rot = rot_enc
	# set_bot_movement(client_id, wheel_joints, 0, 0, 3)
	# while(1):
	# 	x_enc, y_enc, rot_enc = wrapper_encoders(client_id)
	# 	total_rot = rot_enc - start_rot
	# 	if abs(total_rot) > 0.25:
	# 		break
	# # set_bot_movement(client_id, wheel_joints, 0, 0, 0)
	# target_points = [(1, 1), (1, 7), (7, 1), (7, 7)]
	# task_3_primary(client_id, current_position, target_points, bm_map)
	


	time.sleep(3)


if __name__ == "__main__":

	# Room entry co-ordinate
	rooms_entry = [(2,5), (5,8), (5,2), (3,0)]     # example list of tuples

	###############################################################
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

		# Running student's logic
		theme_implementation_primary(client_id, rooms_entry)

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
		print('\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

	except KeyboardInterrupt:
		print('\n[ERROR] Script interrupted by user!')
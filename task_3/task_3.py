'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 3 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_3.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
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






##############################################################


def init_remote_api_server():

	"""
	Purpose:
	---
	This function should first close any open connections and then start
	communication thread with server i.e. CoppeliaSim.
	
	Input Arguments:
	---
	None
	
	Returns:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable
	
	Example call:
	---
	client_id = init_remote_api_server()
	
	"""

	client_id = -1

	##############	ADD YOUR CODE HERE	##############
	sim.simxFinish(client_id)
	client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

	##################################################

	return client_id


def start_simulation(client_id):

	"""
	Purpose:
	---
	This function should first start the simulation if the connection to server
	i.e. CoppeliaSim was successful and then wait for last command sent to arrive
	at CoppeliaSim server end.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	Returns:
	---
	`return_code` 	:  [ integer ]
		the return code generated from the start running simulation remote API
	
	Example call:
	---
	return_code = start_simulation()
	
	"""
	return_code = -2

	##############	ADD YOUR CODE HERE	##############
	if client_id == -1:
		return return_code

	return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
	
	##################################################

	return return_code


def get_vision_sensor_image(client_id):
	
	"""
	Purpose:
	---
	This function should first get the handle of the Vision Sensor object from the scene.
	After that it should get the Vision Sensor's image array from the CoppeliaSim scene.
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	`vision_sensor_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	`return_code` 			:  [ integer ]
		the return code generated from the remote API
	
	Example call:
	---
	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
	"""


	return_code = 0

	##############	ADD YOUR CODE HERE	##############
	vision_sensor_image = []
	image_resolution = []
	return_code = 0

	return_code, visionSensorHandle =  sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	if return_code == sim.simx_return_ok:
		return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, visionSensorHandle, 0, sim.simx_opmode_blocking)
	
	##################################################

	return vision_sensor_image, image_resolution, return_code


def transform_vision_sensor_image(vision_sensor_image, image_resolution):

	"""
	Purpose:
	---
	This function should:
	1. First convert the vision_sensor_image list to a NumPy array with data-type as uint8.
	2. Since the image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
	the new NumPy array should then be resized to a 3-D (three dimensional) NumPy array.
	3. Change the color of the new image array from BGR to RGB.
	4. Flip the resultant image array about the X-axis.
	The resultant image NumPy array should be returned.
	
	Input Arguments:
	---
	`vision_sensor_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	
	Returns:
	---
	`transformed_image` 	:  [ numpy array ]
		the resultant transformed image array after performing above 4 steps
	
	Example call:
	---
	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	
	"""

	transformed_image = None

	##############	ADD YOUR CODE HERE	##############

	transformed_image = np.array(vision_sensor_image, dtype=np.uint8)
	transformed_image = transformed_image.reshape(image_resolution[0], image_resolution[1], -1)

	#RGB to BGR
	transformed_image = transformed_image[..., ::-1]
	
	#Flip across X-axis
	transformed_image = np.flip(transformed_image, 1)

	##################################################
	
	return transformed_image


def stop_simulation(client_id):
	"""
	Purpose:
	---
	This function should stop the running simulation in CoppeliaSim server.
	NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
		  It is already written in the main function.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	`return_code` 	:  [ integer ]
		the return code generated from the stop running simulation remote API
	
	Example call:
	---
	return_code = stop_simulation()
	
	"""

	return_code = -2

	##############	ADD YOUR CODE HERE	##############
	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)

	##################################################

	return return_code


def exit_remote_api_server(client_id):
	
	"""
	Purpose:
	---
	This function should wait for the last command sent to arrive at the Coppeliasim server
	before closing the connection and then end the communication thread with server
	i.e. CoppeliaSim using simxFinish Remote API.
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	None
	
	Example call:
	---
	exit_remote_api_server()
	
	"""

	##############	ADD YOUR CODE HERE	##############
	sim.simxFinish(client_id)

	##################################################


def detect_qr_codes(transformed_image):
	
	"""
	Purpose:
	---
	This function receives the transformed image from the vision sensor and detects qr codes in the image

	Input Arguments:
	---
	`transformed_image` 	:  [ numpy array ]
		the transformed image array
	
	Returns:
	---
	None
	
	Example call:
	---
	detect_qr_codes()
	
	"""

	##############	ADD YOUR CODE HERE	##############
	qr_codes_data = []
	qr_codes_position = []

	detected_qr_codes = decode(transformed_image)
	# print(detected_qr_codes)
	for qr_code in detected_qr_codes:
		data = qr_code.data.decode()
		(x, y, w, h) = qr_code.rect

		center_x = (x + (x + w)) / 2
		center_y = (y + (y + h)) / 2
		qr_codes_data.append(data)
		x = np.float32(np.interp(center_x, [0, 511], [-0.15, 0.15]))
		y = np.float32(np.interp(center_y, [0, 511], [-0.15, 0.15]))
		qr_codes_position.append((x, y))
	##################################################
	
	return qr_codes_data, qr_codes_position


def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):

	"""
	Purpose:
	---
	This function takes desired forward/back, left/right, rotational velocites of the bot as input arguments.
	It should then convert these desired velocities into individual joint velocities(4 joints) and actuate the joints
	accordingly.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	'wheel_joints`      :   [ list]
		Python list containing joint object handles of individual joints

	`forw_back_vel'     :   [ float ]
		Desired forward/back velocity of the bot

	`left_right_vel'    :   [ float ]
		Desired left/back velocity of the bot
	
	`rot_vel'           :   [ float ]
		Desired rotational velocity of the bot
	
	Returns:
	---
	None
	
	Example call:
	---
	set_bot_movement(client_id, wheel_joints, 0.5, 0, 0)
	
	"""

	##############	ADD YOUR CODE HERE	##############
	fl_wheel_handle, fr_wheel_handle, rl_wheel_handle, rr_wheel_handle = wheel_joints

	fr_vel = forw_back_vel + left_right_vel - rot_vel
	fl_vel = forw_back_vel - left_right_vel + rot_vel
	rl_vel = forw_back_vel + left_right_vel + rot_vel
	rr_vel = forw_back_vel - left_right_vel - rot_vel
	

	_ = sim.simxSetJointTargetVelocity(client_id, fr_wheel_handle, fr_vel, sim.simx_opmode_streaming)
	_ = sim.simxSetJointTargetVelocity(client_id, fl_wheel_handle, fl_vel, sim.simx_opmode_streaming)
	_ = sim.simxSetJointTargetVelocity(client_id, rl_wheel_handle, rl_vel, sim.simx_opmode_streaming)
	_ = sim.simxSetJointTargetVelocity(client_id, rr_wheel_handle, rr_vel, sim.simx_opmode_streaming)

	##################################################


def init_setup(client_id):
	
	"""
	Purpose:
	---
	This function will get the object handles of all the four joints in the bot, store them in a list
	and return the list

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	'wheel_joints`      :   [ list]
		Python list containing joint object handles of individual joints
	
	Example call:
	---
	init setup(client_id)
	
	"""

	##############	ADD YOUR CODE HERE	##############
	_, front_left_wheel_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fl', sim.simx_opmode_blocking)
	_, front_right_wheel_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fr', sim.simx_opmode_blocking)
	_, rear_left_wheel_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rl', sim.simx_opmode_blocking)
	_, rear_right_wheel_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rr', sim.simx_opmode_blocking)
	
	wheel_joints = [front_left_wheel_handle, front_right_wheel_handle, rear_left_wheel_handle, rear_right_wheel_handle]

	##################################################

	return wheel_joints


def encoders(client_id):

	"""
	Purpose:
	---
	This function will get the `combined_joint_position` string signal from CoppeliaSim, decode it
	and return a list which contains the total joint position of all joints    

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	'joints_position`      :   [ list]
		Python list containing the total joint position of all joints
	
	Example call:
	---
	encoders(client_id)
	
	"""

	return_code, signal_value=sim.simxGetStringSignal(client_id,'combined_joint_position',sim.simx_opmode_blocking)
	signal_value = signal_value.decode()
	joints_position = signal_value.split("%")

	for index,joint_val in enumerate(joints_position):
		joints_position[index]=float(joint_val)

	return joints_position


def nav_logic():
	"""
	Purpose:
	---
	This function should implement your navigation logic. 
	"""


def shortest_path(source, destination):
	"""
	Purpose:
	---
	This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
	"""
	grid = {}
	e = 0 
	for x in range(0, 9):
		for y in range(0, 13):
			grid[(x, y)] = []
			for i in range(0, 9):
				if i != x:
					grid[(x, y)].append((i, y))
					e += 1
			for j in range(0, 13):
				if j != y:
					grid[(x, y)].append((x, j))
					e += 1
	# print(e)
	visited ={}
	distance = {}
	predecessor = {}
	for x in range(9):
		for y in range(13):
			visited[(x, y)] = False
			distance[(x, y)] = 100000
			predecessor[(x, y)] = -1

	reached = False
	queue = []

	queue.append(source)
	visited[source] = True
	distance[source] = 0
	while queue:
		u = queue[0]
		queue.pop(0)

		for v in grid[u]:
			if visited[v] == False:
				visited[v] = True
				distance[v] = distance[u] + 1
				predecessor[v] = u
				queue.append(v)

				if v == destination:
					reached = True
					break
		if reached:
			break
	
	path = []
	crawl = destination
	path.append(crawl)
	while predecessor[crawl] != -1:
		crawl = predecessor[crawl]
		path.append(crawl)

	path.reverse()
	return path

def task_3_primary(client_id, target_points):
	
	"""
	Purpose:
	---
	
	# NOTE:This is the only function that is called from the main function and from the executable.
	
	Make sure to call all the necessary functions (apart from the ones called in the main) according to your logic. 
	The bot should traverses all the target navigational co-ordinates.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	`target_points`     : [ list ]
		List of tuples where tuples are the target navigational co-ordinates.
	
	Returns:
	---
	
	Example call:
	---
	target_points(client_id, target_points)
	
	"""
	wheel_joints = init_setup(client_id)
	print("............................")
	source = (0, 0)
	for destination in target_points:
		path = shortest_path(source, destination)
		print(path)

		current = path[0]
		path.pop(0) # removing start point
		target = path[0]
		path.pop(0)

		print(current, " --- ", target)
		mode = 1
		speed = 10
		while(1):
			joints_position = encoders(client_id)
			enc_fl, enc_rl, enc_rr, enc_fr = joints_position
			
			#Distance in CM
			enc_fl *= 0.01
			enc_rl *= 0.01
			enc_fr *= 0.01
			enc_rr *= 0.01
			
			x_enc = ((enc_fl + enc_rr) - (enc_fr + enc_rl)) / 4
			y_enc = (enc_fl + enc_rr + enc_fr + enc_rl) / 4
			rot_enc = (((enc_fl + enc_rl) - (enc_fr + enc_rr)) / 4)

			k_p_rot = -300
			rot_velocity = 0 + (k_p_rot * rot_enc)
			print(rot_enc, rot_velocity)

			vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
			transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
			qr_codes_data, qr_codes_position = detect_qr_codes(transformed_image)
			
			
			if current[0] == target[0]:
				if current[1] < target[1]:
					direction = 'up'
				else:
					direction = 'down'
			elif current[1] == target[1]:
				if current[0] < target[0]:
					direction = 'right'
				else:
					direction = 'left'
			print(direction)

			print(qr_codes_data)

			print()

			if mode == 1:
				if str(target) in qr_codes_data:
					mode = 2
					continue
									
				if len(qr_codes_data):
					x_qr, y_qr = qr_codes_position[0]
					QR = [x_enc - x_qr*100, y_enc - y_qr*100]
					k_p = -40
					v_x = k_p * x_qr
					v_y = k_p * y_qr
				else:
					v_x = 0
					v_y = 0

				if direction == 'up':
					if str((target[0], target[1] - 1)) in qr_codes_data:
						speed = 5
					velocity_y = speed
					velocity_x = v_x
				elif direction == 'down':
					if str((target[0], target[1] + 1)) in qr_codes_data:
						speed = 5
					velocity_y = -speed
					velocity_x = v_x
				elif direction == 'left':
					if str((target[0] + 1, target[1])) in qr_codes_data:
						speed = 5
					velocity_y = v_y
					velocity_x = speed
				elif direction == 'right':
					if str((target[0] - 1, target[1])) in qr_codes_data:
						speed = 5
					velocity_y = v_y
					velocity_x = -speed
					
				set_bot_movement(client_id, wheel_joints, velocity_y, velocity_x, rot_velocity)
			elif mode == 2:
				if not len(qr_codes_data):
					continue
				x_qr, y_qr = qr_codes_position[0]
				k_p = -40
				v_x = k_p * x_qr
				v_y = k_p * y_qr
				set_bot_movement(client_id, wheel_joints, v_y, v_x, rot_velocity)
				if (abs(x_qr) < 0.02) and (abs(y_qr) < 0.02):
					print(x_qr, y_qr, mode)
					set_bot_movement(client_id, wheel_joints, 0, 0, rot_velocity)
					if len(path) == 0:
						break

					current = target
					target = path.pop(0)

					speed = 10
					print(current, " --- ", target)
					mode = 1
		
		source = destination
	

	# mode = 1
	# while(1):	
	# 	joints_position = encoders(client_id)
	# 	enc_fl, enc_rl, enc_rr, enc_fr = joints_position

	# 	#Distance in CM
	# 	enc_fl *= 10
	# 	enc_rl *= 10
	# 	enc_fr *= 10
	# 	enc_rr *= 10
		


	# 	x_enc = ((enc_fl + enc_rr) - (enc_fr + enc_rl)) / 4
	# 	y_enc = (enc_fl + enc_rr + enc_fr + enc_rl) / 4
	# 	rot_enc = ((enc_fl + enc_rl) - (enc_fr + enc_rr)) / 4
		

	# 	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
	# 	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	# 	qr_codes_data, qr_codes_position = detect_qr_codes(transformed_image)


	# 	speed = 5
	# 	# print(mode)
	# 	# print(qr_codes_data)
	# 	# print(round(x_enc, 2), round(y_enc, 2), round(rot_enc, 2))
	# 	print(qr_codes_data)
	# 	if mode == 1:
	# 		if '(4, 0)' in qr_codes_data:
	# 			mode = 2
	# 		if len(qr_codes_data):
	# 			x_qr, y_qr = qr_codes_position[0]
	# 			QR = [x_enc - x_qr*100, y_enc - y_qr*100]
	# 			k_p = -40
	# 			v_x = k_p * x_qr
	# 			v_y = k_p * y_qr
	# 			# print(x_enc, y_enc)
	# 		else:
	# 			# print()
	# 			v_x = 0
	# 			v_y = 0
	# 		set_bot_movement(client_id, wheel_joints, v_y, -speed, 0)
	# 	elif mode == 2:
	# 		if not len(qr_codes_data):
	# 			continue
	# 		x_qr, y_qr = qr_codes_position[0]
	# 		k_p = -40
	# 		v_x = k_p * x_qr
	# 		v_y = k_p * y_qr
	# 		set_bot_movement(client_id, wheel_joints, v_y, v_x, 0)
	# 		if (abs(x_qr) < 0.02) and (abs(y_qr) < 0.02):
	# 			print(x_qr, y_qr, mode)
	# 			set_bot_movement(client_id, wheel_joints, 0, 0, 0)
	# 			mode = 3
	# 	elif mode == 3:
	# 		if '(4, 4)' in qr_codes_data:
	# 			mode = 4
	# 		if len(qr_codes_data):
	# 			x_qr, y_qr = qr_codes_position[0]
	# 			QR = [x_enc - x_qr*100, y_enc - y_qr*100]
	# 			k_p = -40
	# 			v_x = k_p * x_qr
	# 			v_y = k_p * y_qr
	# 			# print(x_enc, y_enc)
	# 		else:
	# 			# print()
	# 			v_x = 0
	# 			v_y = 0
	# 		set_bot_movement(client_id, wheel_joints, speed, v_x, 0)
	# 	elif mode == 4:
	# 		if not len(qr_codes_data):
	# 			continue
	# 		x_qr, y_qr = qr_codes_position[0]
	# 		k_p = -40
	# 		v_x = k_p * x_qr
	# 		v_y = k_p * y_qr
	# 		set_bot_movement(client_id, wheel_joints, v_y, v_x, 0)
	# 		print()
	# 		if (abs(x_qr) < 0.02) and (abs(y_qr) < 0.02):
	# 			print(x_qr, y_qr, mode)
	# 			set_bot_movement(client_id, wheel_joints, 0, 0, 0)
	# 			mode = 5
	# 	elif mode == 5:
	# 		if '(0, 4)' in qr_codes_data:
	# 			mode = 6
	# 		if len(qr_codes_data):
	# 			x_qr, y_qr = qr_codes_position[0]
	# 			QR = [x_enc - x_qr*100, y_enc - y_qr*100]
	# 			k_p = -40
	# 			v_x = k_p * x_qr
	# 			v_y = k_p * y_qr
	# 			# print(x_enc, y_enc)
	# 		else:
	# 			# print()
	# 			v_x = 0
	# 			v_y = 0
	# 		set_bot_movement(client_id, wheel_joints, v_y, speed, 0)
	# 	elif mode == 6:
	# 		if not len(qr_codes_data):
	# 			continue
	# 		x_qr, y_qr = qr_codes_position[0]
	# 		k_p = -40
	# 		v_x = k_p * x_qr
	# 		v_y = k_p * y_qr
	# 		set_bot_movement(client_id, wheel_joints, v_y, v_x, 0)
	# 		if (abs(x_qr) < 0.02) and (abs(y_qr) < 0.02):
	# 			print(x_qr, y_qr, mode)
	# 			set_bot_movement(client_id, wheel_joints, 0, 0, 0)
	# 			mode = 7
	# 	elif mode == 7:
	# 		if '(0, 0)' in qr_codes_data:
	# 			mode = 8
	# 		if len(qr_codes_data):
	# 			x_qr, y_qr = qr_codes_position[0]
	# 			QR = [x_enc - x_qr*100, y_enc - y_qr*100]
	# 			k_p = -40
	# 			v_x = k_p * x_qr
	# 			v_y = k_p * y_qr
	# 			# print(x_enc, y_enc)
	# 		else:
	# 			# print()
	# 			v_x = 0
	# 			v_y = 0
	# 		set_bot_movement(client_id, wheel_joints, -speed, v_x, 0)
	# 	elif mode == 8:
	# 		if not len(qr_codes_data):
	# 			continue
	# 		x_qr, y_qr = qr_codes_position[0]
	# 		k_p = -40
	# 		v_x = k_p * x_qr
	# 		v_y = k_p * y_qr
	# 		set_bot_movement(client_id, wheel_joints, v_y, v_x, 0)
	# 		if (abs(x_qr) < 0.02) and (abs(y_qr) < 0.02):
	# 			print(x_qr, y_qr, mode)
	# 			set_bot_movement(client_id, wheel_joints, 0, 0, 0)
	# 			mode = 1

	set_bot_movement(client_id, wheel_joints, 0, 0, 0)

	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
	print(image_resolution, return_code)

	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	# cv2.imshow("ortho", transformed_image)

	# qr_codes = detect_qr_codes(transformed_image)
	# print(qr_codes)
		

	# cv2.waitKey(0)
	# cv2.destroyAllWindows()




if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(4, 5), (5, 8), (2, 8), (1, 5)]    # You can give any number of different co-ordinates


	##################################################
	## NOTE: You are NOT allowed to make any changes in the code below ##

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

		task_3_primary(client_id, target_points)
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
		print('\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()
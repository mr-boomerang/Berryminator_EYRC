'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 2A of Berryminator(BM) Theme (eYRC 2021-22).
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
# Filename:			task_2a.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os, sys
import traceback
##############################################################

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


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
# Helper variables for finding color of the contour

red = np.uint8([[[0, 0, 255]]])  # RGB for Red
hsv_red = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)[0][0] # HSV for Red
#Defines the HSV ranges for the colro RED
lower_red_1 = np.array([hsv_red[0], 100, 100])
upper_red_1 = np.array([hsv_red[0] + 10, 255, 255])
lower_red_2 = np.array([180 - hsv_red[0] - 10, 100, 100])
upper_red_2 = np.array([180 + hsv_red[0], 255, 255])

blue = np.uint8([[[255, 0, 0]]]) # RGB for Blue
hsv_blue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)[0][0] # HSV for Blue
#Defines the HSV ranges for the color BLUE
lower_blue = np.array([hsv_blue[0] - 10, 100, 100])
upper_blue = np.array([hsv_blue[0] + 10, 255, 255])


yellow = np.uint8([[[0, 255, 255]]]) # RGB for yellow
hsv_yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)[0][0] # HSV for Yellow
#Defines the HSV Ranges for the color YELLOW
lower_yellow = np.array([hsv_yellow[0] - 10, 100, 100])
upper_yellow = np.array([hsv_yellow[0] + 10, 255, 255])

##############################################################

def get_vision_sensor_image(client_id, vision_sensor_handle):
	
	"""
	Purpose:
	---
	This function takes the client id and handle of the vision sensor scene object as input
	arguments and returns the vision sensor's image array from the CoppeliaSim scene.

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`vision_sensor_handle`    :   [ integer ]
		the handle of the vision sensor scene object
	
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

	vision_sensor_image = []
	image_resolution = []
	return_code = 0

	##############	ADD YOUR CODE HERE	##############
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vision_sensor_handle, 0, sim.simx_opmode_blocking)

	##################################################

	return vision_sensor_image, image_resolution, return_code

def get_vision_sensor_depth_image(client_id, vision_sensor_handle):
	
	"""
	Purpose:
	---
	This function takes the client id and handle of the vision sensor scene object as input
	arguments and returns the vision sensor's depth buffer array from the CoppeliaSim scene.
	Input Arguments:
	---
	`client_id`               :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`vision_sensor_handle`    :   [ integer ]
		the handle of the vision sensor scene object
	
	Returns:
	---
	`vision_sensor_depth_image` 	:  [ list ]
		the depth buffer array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	`return_code` 			:  [ integer ]
		the return code generated from the remote API
	
	Example call:
	---
	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
	"""

	vision_sensor_depth_image = []
	image_resolution = []
	return_code = 0

	##############	ADD YOUR CODE HERE	##############
	return_code, image_resolution, vision_sensor_depth_image = sim.simxGetVisionSensorDepthBuffer(client_id, vision_sensor_handle, sim.simx_opmode_blocking)

	##################################################

	return vision_sensor_depth_image, image_resolution, return_code

def transform_vision_sensor_depth_image(vision_sensor_depth_image, image_resolution):

	"""
	Purpose:
	---
	This function converts the depth buffer array received from vision sensor and converts it into
	a numpy array that can be processed by OpenCV
	This function should:
	1. First convert the vision_sensor_depth_image list to a NumPy array with data-type as float32.
	2. Since the depth image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
	the new NumPy array should then be resized to a 2-D (two dimensional) NumPy array.
	3. Flip the resultant image array about the appropriate axis. The resultant image NumPy array should be returned.
	
	Input Arguments:
	---
	`vision_sensor_depth_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get_vision_sensor_depth_image() function
	
	Returns:
	---
	`transformed_depth_image` 	:  [ numpy array ]
		the resultant transformed image array after performing above 3 steps
	
	Example call:
	---
	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	
	"""

	transformed_depth_image = None

	##############	ADD YOUR CODE HERE	##############
	# Converting list to Numpy Array of dtype float32
	transformed_depth_image = np.array(vision_sensor_depth_image, dtype=np.float32)
	
	# Reshaping the numpy array from 1D to 2D
	transformed_depth_image = transformed_depth_image.reshape(image_resolution[0], image_resolution[1], -1)
	
	# Flipping the image across Y axis
	transformed_depth_image = np.flip(transformed_depth_image, 1)

	##################################################
	
	return transformed_depth_image

def detect_berries(transformed_image, transformed_depth_image):
	"""
	Purpose:
	---
	This function takes the transformed image and transformed depth image as input arguments and returns
	the pixel coordinates and depth values in form of a dictionary.
	
	Input Arguments:
	---
 	`transformed_image` 	:  [ numpy array ]
 		the transformed image array
 	`transformed_depth_image` 	:  [ numpy array ]
 		the transformed depth image array
	
	Returns:
	---
	`berries_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of all the berries
	
	Example call:
	---
	berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
	
	"""
	berries_dictionary = {}
	berries = ["Strawberry", "Blueberry", "Lemon"]

	##############	ADD YOUR CODE HERE	##############
	# Initialize the Dictionary with empty lists for each berry type
	for berry_type in berries:
		berries_dictionary[berry_type] = []

	gray = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)
	_, threshold = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)

	contours, _ = cv2.findContours(
		threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	for contour in contours:
		## Finding Shape
		epsilon = 0.03 * cv2.arcLength(contour, True)
		approx = cv2.approxPolyDP(
			contour, epsilon, True)
		num_points = len(approx)

		if num_points < 6: 		#Only allow circles to pass through
			continue
		
		## Finding Center
		M = cv2.moments(contour)
		centroid_x = int(M['m10'] / M['m00'])
		centroid_y = int(M['m01'] / M['m00'])

		## Creating Mask for individual contour
		mask = np.zeros(gray.shape, np.uint8)
		cv2.drawContours(mask, [contour], 0, (255, 255, 255), -1)

		## Finding Depth
		depth = cv2.mean(transformed_depth_image, mask = mask) # Average of the depth values after applying mask!!
		depth = depth[0]

		## Finding Color
		rgb_color = cv2.mean(transformed_image, mask = mask)
		rgb_color = np.uint8([[rgb_color[:-1]]])
		hsv_color = cv2.cvtColor(rgb_color, cv2.COLOR_BGR2HSV)

		is_blue = cv2.inRange(hsv_color, lower_blue, upper_blue)[0][0]
		is_red = cv2.bitwise_or(cv2.inRange(hsv_color, lower_red_1, upper_red_1), cv2.inRange(hsv_color, lower_red_2, upper_red_2))[0][0]
		is_yellow = cv2.inRange(hsv_color, lower_yellow, upper_yellow)[0][0]
		
		if is_red:
			berry_type = berries[0] #Strawberry
		elif is_blue:
			berry_type = berries[1] #Blueberry
		elif is_yellow:
			berry_type = berries[2] #Lemon
		else:
			continue # Skip for any other color
		 
		berries_dictionary[berry_type].append((centroid_x, centroid_y, depth))

	##################################################
	return berries_dictionary

def detect_berry_positions(berries_dictionary):
	"""
	Purpose:
	---
	This function takes the berries_dictionary as input arguments and calculates the 3D positions of the
	berries with respect to vision sensor. The final output is returned in another dictionary.
	
	Input Arguments:
	---
	`berries_dictionary` 	:  [ dictionary ]
		the dictionary returned by detect_berries() function
	
	Returns:
	---
	`berry_positions_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of 3D positions of all the berries
	
	Example call:
	---
	berry_positions_dictionary = detect_berry_positions(berries_dictionary)
	
	"""
	berry_positions_dictionary = {}
	berries = ["Strawberry", "Blueberry", "Lemon"]

	##############	ADD YOUR CODE HERE	##############
	# Initialize the Dictionary with empty lists for each berry type
	for berry_type in berries:
		berry_positions_dictionary[berry_type] = []

	for berry_type in berries:
		for (centroid_x, centroid_y, depth) in berries_dictionary[berry_type]:
			#Extrapolating Values using np.interp
			h = np.float32(np.interp(centroid_x, [0, 511], [-0.5, 0.5]))
			v = np.float32(np.interp(centroid_y, [0, 511], [-0.5, 0.5]))
			dis = np.float32(np.interp(depth, [0, 1], [0.01, 2.00e+0]))
			
			dis += 0.0125	## Adding 0.0125 as the average depth will be (Radius/2) metres nearer to the camera. Radius of each berry is 0.25m. 

			## Rounding to 2 decimal places gives exactly matches the Actual Berry Positions obtained in the task_2a_outputs.txt file.
			## However, the code passes all the tests even without rounding.
			# h = np.round(h, 2)
			# v = np.round(v, 2)
			# dis = np.round(dis, 2)

			coord_3d = (h, v, dis)
			berry_positions_dictionary[berry_type].append(coord_3d)
			
	##################################################

	return berry_positions_dictionary

def get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary):
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
	"""
	Purpose:
	---
	This function takes the transformed_image and the dictionaries returned by detect_berries()
	and  detect_berry_positions() functions. This function is already completed for your reference
	and will be helpful for debugging purposes.

	Input Arguments:
	---
	`transformed_image` :	[ numpy array ]
			numpy array of image returned by cv2 library

	`berries_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of all the berries

	`berry_positions_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of 3D positions of all the berries

	Returns:
	---
	`labelled_image` :	[ numpy array ]
			labelled image
	
	Example call:
	---
	transformed_image = get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)
	"""
	labelled_image = np.array(transformed_image)
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

	for berry_type in berries_dictionary.keys():
		berry_details_list = berries_dictionary[berry_type]
		berry_positions_list = berry_positions_dictionary[berry_type]
		for index in range(len(berry_details_list)):
			pixel_x, pixel_y, depth_val = berry_details_list[index]
			coordinates = (pixel_x, pixel_y)
			horizontal_displacement, vertical_displacement, distance_from_sensor = berry_positions_list[index]
			horizontal_displacement, vertical_displacement, distance_from_sensor = round(horizontal_displacement, 2), round(vertical_displacement, 2), round(distance_from_sensor, 2)
			cv2.putText(labelled_image, str((horizontal_displacement, vertical_displacement, distance_from_sensor)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)
	return labelled_image


if __name__ == "__main__":

	berries_dictionary = {}
	berry_positions_dictionary = {}

	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')
	cv2.namedWindow('transformed image', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('transformed depth image', cv2.WINDOW_AUTOSIZE)

	try:
		# Initiate Remote API connection
		client_id = task_1b.init_remote_api_server()

		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')
			return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor', sim.simx_opmode_blocking)

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

	while True:

	# Get image array and depth buffer from vision sensor in CoppeliaSim scene
		try:
			vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id, vision_sensor_handle)
			vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(client_id, vision_sensor_handle)

			if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
				print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')

				# Get the transformed vision sensor image captured in correct format
				try:
					transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
					transformed_depth_image = transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)

					if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):

						berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
						print("Berries Dictionary = ", berries_dictionary)
						berry_positions_dictionary = detect_berry_positions(berries_dictionary)
						print("Berry Positions Dictionary = ",berry_positions_dictionary)

						labelled_image = get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)






						cv2.imshow('transformed image', transformed_image)
						cv2.imshow('transformed depth image', transformed_depth_image)
						cv2.imshow('labelled image', labelled_image)
						if cv2.waitKey(1) & 0xFF == ord('q'):
							break
						

					else:
						print('\n[ERROR] transform_vision_sensor_image function is not configured correctly, check the code.')
						print('Stop the CoppeliaSim simulation manually.')
						print()
						sys.exit()

				except Exception:
					print('\n[ERROR] Your transform_vision_sensor_image function throwed an Exception, kindly debug your code!')
					print('Stop the CoppeliaSim simulation manually.\n')
					traceback.print_exc(file=sys.stdout)
					print()
					sys.exit()

			else:
				print('\n[ERROR] get_vision_sensor function is not configured correctly, check the code.')
				print('Stop the CoppeliaSim simulation manually.')
				print()
				sys.exit()

		except Exception:
			print('\n[ERROR] Your get_vision_sensor_image function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()
		
		
	cv2.destroyAllWindows()

	# Ending the Simulation
	try:
		return_code = task_1b.stop_simulation(client_id)
		
		if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
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




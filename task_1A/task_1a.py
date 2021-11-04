'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 1A of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ BM_1707 ]
# Author List:		[ Parth Shah, Shubhankar Riswadkar, Chirag Jain, Bhavya Vira ]
# Filename:			task_1a.py
# Functions:		detect_shapes
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################

blue = np.uint8([[[255, 0, 0]]])
hsv_blue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)[0][0]
lower_blue = np.array([hsv_blue[0] - 10, 100, 100])
upper_blue = np.array([hsv_blue[0] + 10, 255, 255])

green = np.uint8([[[0, 255, 0]]])
hsv_green = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)[0][0]
lower_green = np.array([hsv_green[0] - 10, 100, 100])
upper_green = np.array([hsv_green[0] + 10, 255, 255])

red = np.uint8([[[0, 0, 255]]])
hsv_red = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)[0][0]
lower_red_1 = np.array([hsv_red[0], 100, 100])
upper_red_1 = np.array([hsv_red[0] + 10, 255, 255])
lower_red_2 = np.array([180 - hsv_red[0] - 10, 100, 100])
upper_red_2 = np.array([180 + hsv_red[0], 255, 255])

orange = np.uint8([[[0, 150, 255]]])
hsv_orange = cv2.cvtColor(orange, cv2.COLOR_BGR2HSV)[0][0]
lower_orange = np.array([hsv_orange[0] - 10, 100, 100])
upper_orange = np.array([hsv_orange[0] + 10, 255, 255])

##############################################################

def detect_shapes(img):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a nested list
	containing details of colored (non-white) shapes in that image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	Returns:
	---
	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image
	
	Example call:
	---
	shapes = detect_shapes(img)
	"""    
	detected_shapes = []

	##############	ADD YOUR CODE HERE	##############
	#Converting BGR to Grayscale
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	
	#Thresholding image to make all shapes of same color
	_, threshold = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

	#Finding all Contours
	contours, hierarchy = cv2.findContours(
		threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	is_outer  = 0
	for contour in contours:
		#First Contour detected is always the entire image boundary, we don't require that!!
		if is_outer == 0:
			is_outer = 1
			continue

		shape = None
		color = None
		center = None
		
		## Finding Shape
		epsilon = 0.03 * cv2.arcLength(contour, True)
		approx = cv2.approxPolyDP(
			contour, epsilon, True)

		num_points = len(approx)
		if num_points == 3:
			shape = 'Triangle'
		elif num_points == 4:
			(x, y, w, h) = cv2.boundingRect(approx)
			ratio = w / h
			if ratio > 0.95 and ratio < 1.05:
				shape = 'Square'
			else:
				shape = 'Rectangle'
		elif num_points == 5:
			shape = 'Pentagon'
		elif num_points > 6:
			shape = 'Circle'
		# print('Shape - ', shape)

		## Finding Center
		M = cv2.moments(contour)
		center_x = int(M['m10'] / M['m00'])
		center_y = int(M['m01'] / M['m00'])
		center = (center_x, center_y)
		# print('Center - ', center)

		## Finding Color
		mask = np.zeros(gray.shape, np.uint8)
		cv2.drawContours(mask, [contour], 0, (255, 255, 255), -1)
	
		rgb_color = cv2.mean(img, mask = mask)
		rgb_color = np.uint8([[rgb_color[:-1]]])
		hsv_color = cv2.cvtColor(rgb_color, cv2.COLOR_BGR2HSV)
		
		is_blue = cv2.inRange(hsv_color, lower_blue, upper_blue)[0][0]
		is_green = cv2.inRange(hsv_color, lower_green, upper_green)[0][0]
		is_red = cv2.bitwise_or(cv2.inRange(hsv_color, lower_red_1, upper_red_1), cv2.inRange(hsv_color, lower_red_2, upper_red_2))[0][0]
		is_orange = cv2.inRange(hsv_color, lower_orange, upper_orange)[0][0]
		
		if is_blue:
			color = 'Blue'
		elif is_green:
			color = 'Green'
		elif is_red:
			color = 'Red'
		elif is_orange:
			color = 'Orange'
		
		# print('Color - ', color)
		# print('---')
		detected_shapes.append([color, shape, center])
	##################################################
	
	return detected_shapes

def get_labeled_image(img, detected_shapes):
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
	"""
	Purpose:
	---
	This function takes the image and the detected shapes list as an argument
	and returns a labelled image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image

	Returns:
	---
	`img` :	[ numpy array ]
			labelled image
	
	Example call:
	---
	img = get_labeled_image(img, detected_shapes)
	"""
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

	for detected in detected_shapes:
		colour = detected[0]
		shape = detected[1]
		coordinates = detected[2]
		cv2.putText(img, str((colour, shape)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
	return img

if __name__ == '__main__':
	
	# path directory of images in 'test_images' folder
	img_dir_path = 'test_images/'

	# path to 'test_image_1.png' image file
	file_num = 1
	img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
	
	# read image using opencv
	img = cv2.imread(img_file_path)
	
	print('\n============================================')
	print('\nFor test_image_' + str(file_num) + '.png')
	
	# detect shape properties from image
	detected_shapes = detect_shapes(img)
	print(detected_shapes)
	
	# display image with labeled shapes
	img = get_labeled_image(img, detected_shapes)
	cv2.imshow("labeled_image", img)
	cv2.waitKey(2000)
	cv2.destroyAllWindows()
	
	choice = input('\nDo you want to run your script on all test images ? => "y" or "n": ')
	
	if choice == 'y':

		for file_num in range(1, 15):
			
			# path to test image file
			img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
			
			# read image using opencv
			img = cv2.imread(img_file_path)
	
			print('\n============================================')
			print('\nFor test_image_' + str(file_num) + '.png')
			
			# detect shape properties from image
			detected_shapes = detect_shapes(img)
			print(detected_shapes)
			
			# display image with labeled shapes
			img = get_labeled_image(img, detected_shapes)
			cv2.imshow("labeled_image", img)
			cv2.waitKey(2000)
			cv2.destroyAllWindows()



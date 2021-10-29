import cv2
import numpy as np
import os

img = cv2.imread('./test_images/test_image_15.png')
# cv2.imshow('shapes', img)
# cv2.waitKey(0)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# cv2.imshow('shapes', gray)
# cv2.waitKey(0)

_, threshold = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

# cv2.imshow('shapes', threshold)
# cv2.waitKey(0)

contours, hierarchy = cv2.findContours(
    threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


#######
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
#######

is_outer  = 0
for contour in contours:
    if is_outer == 0:
        is_outer = 1
        continue

    shape = None
    color = None
    center = None
    
    ## Finding Shape
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(
        contour, epsilon, True)

    num_points = len(approx)
    if num_points == 3:
        shape = "triangle"
    elif num_points == 4:
        (x, y, w, h) = cv2.boundingRect(approx)
        ratio = w / h
        if ratio > 0.95 and ratio < 1.05:
            shape = "square"
        else:
            shape = "rectangle"
    elif num_points == 5:
        shape = "pentagon"
    elif num_points > 6:
        shape = "circle"
    print("Shape - ", shape)

    ## Finding Center
    M = cv2.moments(contour)
    center_x = int(M['m10'] / M['m00'])
    center_y = int(M['m01'] / M['m00'])
    center = (center_x, center_y)
    print("Center - ", center)

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
        color = "blue"
    elif is_green:
        color = "green"
    elif is_red:
        color = "red"
    elif is_orange:
        color = "orange"
    
    print("Color - ", color)
    print("-----")
    # cv2.imshow('shapes', mask)
    # cv2.waitKey(0)

cv2.imshow('shapes', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
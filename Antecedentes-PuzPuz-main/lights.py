#!/usr/bin/env python
# Actividad 5 Detectar colores de semaforo y activar motores
# Convertir de imagen de ROS a imagen de OpenCv y viceversa

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Load image
path = r'/home/noemi/puzzlebot/vision/src/traffic_light/imgs/colores.jpeg'
cv_image = cv2.imread(path) # Read img

#Convert the BGR image to HSV colour space
hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

#Set the lower and upper bounds for the green hue
lower_green = np.array([35,30,110])
upper_green = np.array([60,255,255])

#Set the lower and upper bounds for the red hue
lower_red = np.array([0,100,180])
upper_red = np.array([220,255,255])

#Create a mask for red and green colour using inRange function
red_mask = cv2.inRange(hsv, lower_red, upper_red)
green_mask = cv2.inRange(hsv, lower_green, upper_green)

#Perform bitwise and on the original image arrays using the mask
res_red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
res_green = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

#Invert black and white
invertR = cv2.bitwise_not(red_mask)
ret, thR = cv2.threshold(invertR, 120, 255, cv2.THRESH_TOZERO)
invertG = cv2.bitwise_not(green_mask)
ret, thG = cv2.threshold(invertG, 120, 255, cv2.THRESH_TOZERO)

##########################################################################
#Eliminate small blobs
r_contours, r_hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
g_contours, g_hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
threshold_blob_area = 10000

for i in range(1, len(r_contours)):
  index_level = int(r_hierarchy[0][i][1])
  if index_level <= i:
    cnt = r_contours[i]
    area = cv2.contourArea(cnt)
    print(area)
  if(area) <= threshold_blob_area:
    cv2.drawContours(red_mask, [cnt], -1, 0, -1, 1)

for i in range(1, len(g_contours)):
  index_level = int(g_hierarchy[0][i][1])
  if index_level <= i:
    cnt = g_contours[i]
    area = cv2.contourArea(cnt)
    print(area)
  if(area) <= threshold_blob_area:
    cv2.drawContours(green_mask, [cnt], -1, 0, -1, 1)

#Find contour of a blob
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
opening_red = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=4)
opening_green = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=4)

#Get coordinates
xR,yR,wR,hR = cv2.boundingRect(opening_red)
cv2.rectangle(opening_red, (xR, yR), (xR + wR, yR + hR), (139,0,0), 4)
for ind, cont in enumerate(r_contours):
  if(len(cont) > 5):
    (xeR,yeR), (MAR,maR), angleR = cv2.fitEllipse(cont)
    cv2.ellipse(opening_red, (int(xeR),int(yeR)), (int(MAR/2),int(maR/2)), int(angleR), 0,360, (150,150,0),4)
  else:
    pass

xG,yG,wG,hG = cv2.boundingRect(opening_green)
cv2.rectangle(opening_green, (xG, yG), (xG + wG, yG + hG), (139,0,0), 4)
for ind2, cont2 in enumerate(g_contours):
  print(cont2)
  if(len(cont2) > 5):
    (xeG,yeG), (MAG,maG), angleG = cv2.fitEllipse(cont2)
    cv2.ellipse(opening_green, (int(xeG),int(yeG)), (int(MAG/2),int(maG/2)), int(angleG), 0,360, (150,150,0),4)
  else:
    pass

#Display the images
cv2.imshow("Original", cv_image)
cv2.imshow("Res green", res_green)
cv2.imshow("Res red", res_red)
cv2.imshow("Red Mask", red_mask)
cv2.imshow("Green Mask", green_mask)
cv2.imshow("Opening Red", opening_red)
cv2.imshow("Opening Green", opening_green)
cv2.waitKey(3)

#Notify when a color circle is found
if((wR > 100) and (wR<500) and (wG > 100) and (wG<500)):
  print("Encontre circulos de los 2")
elif(wG > 100) and (wG<500):
  print("Encontre un circulo verde", wG)
elif(wR > 100) and (wR<500):
  print("Encontre un circulo rojo", wR)
else:
  print("No hay circulos")

if cv2.waitKey(0):
  cv2.destroyAllWindows()

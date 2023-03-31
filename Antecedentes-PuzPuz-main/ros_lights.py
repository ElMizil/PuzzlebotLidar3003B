#!/usr/bin/env python
# Actividad 5 Detectar colores de semaforo, activar motores y seguir punto
# Convertir de imagen de ROS a imagen de OpenCv y viceversa

# Libraries
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

''' Clase para filtrado e identificacion de imagenes ''''
class image_converter:
  # Publishers y subscribers
  def __init__(self):
    self.image_pub = rospy.Publisher("Video",Image, queue_size = 10)
    self.msgs_motores = rospy.Publisher("Mensaje_semaforo", String, queue_size = 10)
    self.pos = rospy.Publisher("Posicion_semaforo", Int32, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #Convert the BGR image to HSV colour space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #Set the lower and upper bounds for the green hue
    lower_green = np.array([35,30,110])
    upper_green = np.array([60,255,255])

    #Set the lower and upper bounds for the red hue
    lower_red = np.array([0,100,180])
    upper_red = np.array([220,255,255])

    #Set the lower and upper bounds for the yellow hue
    lower_yellow = np.array([20,100,120])
    upper_yellow = np.array([40,255,255])

    #Create a mask for red and green colour using inRange function
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #Perform bitwise and on the original image arrays using the mask
    res_red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    res_green = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
    res_yellow = cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)

    #Invert black and white
    invertR = cv2.bitwise_not(red_mask)
    ret, thR = cv2.threshold(invertR, 120, 255, cv2.THRESH_TOZERO)
    invertG = cv2.bitwise_not(green_mask)
    ret, thG = cv2.threshold(invertG, 120, 255, cv2.THRESH_TOZERO)
    invertY = cv2.bitwise_not(yellow_mask)
    ret, thY = cv2.threshold(invertY, 120, 255, cv2.THRESH_TOZERO)

    ##########################################################################
    #Eliminate small blobs
    r_contours, r_hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    g_contours, g_hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    threshold_blob_area = 5000

    #Red Noise Blobs
    for i in range(1, len(r_contours)):
      index_level = int(r_hierarchy[0][i][1])
      if index_level <= i:
        cnt = r_contours[i]
        area = cv2.contourArea(cnt)
        print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(red_mask, [cnt], -1, 0, -1, 1)

    #Green Noise Blobs
    for i in range(1, len(g_contours)):
      index_level = int(g_hierarchy[0][i][1])
      if index_level <= i:
        cnt = g_contours[i]
        area = cv2.contourArea(cnt)
        print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(green_mask, [cnt], -1, 0, -1, 1) 

    #Yellow Noise blobs
    for i in range(1, len(y_contours)):
      index_level = int(y_hierarchy[0][i][1])
      if index_level <= i:
        cnt = y_contours[i]
        area = cv2.contourArea(cnt)
        print(area)
        if(area) <= threshold_blob_area:
          cv2.drawContours(yellow_mask, [cnt], -1, 0, -1, 1)

    #Find contour of a blob
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    opening_red = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=4)
    opening_green = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=4)
    opening_yellow = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=4)

    ### Get coordinates ###

    #Red rectangle 
    xR,yR,wR,hR = cv2.boundingRect(opening_red)
    cv2.rectangle(opening_red, (xR, yR), (xR + wR, yR + hR), (139,0,0), 4)

    #Green rectangle
    xG,yG,wG,hG = cv2.boundingRect(opening_green)
    cv2.rectangle(opening_green, (xG, yG), (xG + wG, yG + hG), (139,0,0), 4)

    #Yellow rectangle
    xY,yY,wY,hY = cv2.boundingRect(opening_yellow)
    cv2.rectangle(opening_yellow, (xY, yY), (xY + wY, yY + hY), (139,0,0), 4)
    #Yellow ellipse
    for ind, cont in enumerate(y_contours):
      print(cont)
      if(len(cont) > 5):
        (xeY,yeY), (MAY,maY), angleY = cv2.fitEllipse(cont)
        cv2.ellipse(opening_yellow, (int(xeY),int(yeY)), (int(MAY/2),int(maY/2)), int(angleY), 0,360, (150,150,0),4)
      else:
        pass

    #Display the images
    cv2.imshow("Original", cv_image)
    #cv2.imshow("Res green", res_green)
    #cv2.imshow("Res red", res_red)
    #cv2.imshow("Res yellow", res_yellow)
    #cv2.imshow("Red Mask", red_mask)
    #cv2.imshow("Green Mask", green_mask)  
    #cv2.imshow("Yellow Mask", yellow_mask)  
    cv2.imshow("Opening Red", opening_red)
    cv2.imshow("Opening Green", opening_green)
    cv2.imshow("Opening Yellow", opening_yellow)
    cv2.waitKey(3)

    #Notify when a color circle is found
    if(wR > 5) and (wR<500):
      print("Encontre un circulo rojo")
      self.msgs_motores.publish("Rojo")
      self.pos.publish(xeR)
    elif(wG > 5) and (wG<500):
      print("Encontre un circulo verde")
      self.msgs_motores.publish("Verde")
      self.pos.publish(xeG)
    elif(wY > 5) and (wY<500):
      print("Encontre un circulo amarillo")
      self.msgs_motores.publish("Amarillo")
      self.pos.publish(xeY)
    else:
      print("No hay circulos")

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  # Initialize node
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)

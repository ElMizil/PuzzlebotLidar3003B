#!/usr/bin/env python
#Codigo usado actualmente como demo para pruebas de mover el xarm a una posicion deseada usando vision, Equipo 3
# Convertir de imagen de ROS a imagen de OpenCv y viceversa


import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("Video",Image, queue_size = 10)
    #self.msgs_motores = rospy.Publisher("Mensaje_semaforo", String, queue_size = 10)
    self.msgs_xarm = rospy.Publisher("PosXarm", Float32MultiArray, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #Convert the BGR image to HSV colour space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


    #Set the lower and upper bounds for the red hue
    lower_red1 = np.array([0,100,20])
    upper_red1 = np.array([8,255,255])
    lower_red2 = np.array([175,100,20])
    upper_red2 = np.array([179,255,255])

    #Set the lower and upper bounds for the yellow hue
    lower_yellow = np.array([20,100,120]) #Azul (20, 90, 90)
    upper_yellow = np.array([40,255,255]) #Azul (255, 255, 255)

    #Create a mask for red and green colour using inRange function
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.add(red_mask1,red_mask2)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #Perform bitwise and on the original image arrays using the mask
    res_red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    res_yellow=cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)

    #Invert black and white
    invertR = cv2.bitwise_not(red_mask)
    ret, thR = cv2.threshold(invertR, 120, 255, cv2.THRESH_TOZERO)
    invertY = cv2.bitwise_not(yellow_mask)
    ret, thY = cv2.threshold(invertY, 120, 255, cv2.THRESH_TOZERO)
    

    ##########################################################################
    #Eliminate small blobs
    r_contours, r_hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    y_contours, y_hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    threshold_blob_area = 5000

    for i in range(1, len(r_contours)):
      index_level = int(r_hierarchy[0][i][1])
      if index_level <= i:
        cnt = r_contours[i]
        area = cv2.contourArea(cnt)
        #print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(red_mask, [cnt], -1, 0, -1, 1)

    #Yellow Noise blobs
    for i in range(1, len(y_contours)):
      index_level = int(y_hierarchy[0][i][1])
      if index_level <= i:
        cnt = y_contours[i]
        area = cv2.contourArea(cnt)
        #print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(yellow_mask, [cnt], -1, 0, -1, 1)

    #Find contour of a blob
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    opening_red = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=4)
    opening_yellow = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=4)

    #Get coordinates
    #Red rectangle 
    xR,yR,wR,hR = cv2.boundingRect(opening_red)
    cv2.rectangle(opening_red, (xR, yR), (xR + wR, yR + hR), (139,0,0), 4)
    xY,yY,wY,hY = cv2.boundingRect(opening_yellow)
    cv2.rectangle(opening_yellow, (xY, yY), (xY + wY, yY + hY), (139,0,0), 4)

  
      #cv_image = cv2.resize(cv_image,, interpolation = cv2.INTER_AREA)
      #opening_red = cv2.resize(opening_red,(wR,hR), interpolation = cv2.INTER_AREA)
    
    #Notify when a color circle is found
    if(wR > 5) and (wR<500):
      print("Encontre un circulo rojo")
      #self.msgs_motores.publish("Rojo")
    if(wY > 5) and (wY<500):
      print("Encontre un circulo amarillo")
      #self.msgs_motores.publish("Amarillo")
      list = Float32MultiArray()
      ex = xY-xR
      ey = yY-yR
      list.data = [0.3, 0, 0.5]
      self.msgs_xarm.publish(list)
    else:
      print("No hay circulos") 


    #Recortamos las imagenes para que solo salga el circulo en su mayoria
    #cv_image = cv2.resize(cv_image,(470,470), interpolation = cv2.INTER_AREA)
    #opening_red = cv2.resize(opening_/red,(470,470), interpolation = cv2.INTER_AREA)
    #Display the images
    cv2.imshow("Original", cv_image)
    #cv2.imshow("Res green", res_green)
    #cv2.imshow("Res red", res_red)
    #cv2.imshow("Red Mask", red_mask)
    #cv2.imshow("Green Mask", green_mask)  
    cv2.imshow("Opening Red", opening_red)
    #cv2.imshow("Opening Green", opening_green)
    cv2.waitKey(3)


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  list = Float32MultiArray()
  list.data = [0.3, 0, 0.5]
  # ic.msgs_xarm.publish(list)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)

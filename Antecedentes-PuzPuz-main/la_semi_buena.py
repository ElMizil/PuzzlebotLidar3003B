#!/usr/bin/env python
# Codigo usado actualmente como demo para pruebas de vision, 
# Equipo 3
# Actividad 6 detectar la linea
# Convertir de imagen de ROS a imagen de OpenCv y viceversa

# Libraries
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32, Int32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

class image_converter:

  def __init__(self):
    # Publish and subscribe different topics
    self.image_pub = rospy.Publisher("Video",Image, queue_size = 10)
    self.msgs_motores = rospy.Publisher("Mensaje_semaforo", String, queue_size = 10)
    self.pos = rospy.Publisher("Posicion_semaforo", Int32, queue_size = 10)
    self.width = rospy.Publisher("Width", Int32, queue_size=10)
    self.msgs_seguidor = rospy.Publisher("Seguidor", Int32MultiArray, queue_size=10)
    self.pubvelo = rospy.Publisher("cmd_vel", Twist, queue_size=10) #Topic de cmd_vel para publicar las velocidades

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)
    self.vw = Twist()
    self.vw.linear.x=0
    self.vw.linear.y=0
    self.vw.linear.z=0
    self.vw.angular.x=0
    self.vw.angular.y=0
    self.vw.angular.z=0
    self.size = 300
    self.setPoint = (self.size/2)
    self.error = 0
    self.semafo = "X"
    self.i = 1
    self.aux = False
    self.auxR = False
    #self.r = rospy.Rate(15)
 
  #Funcion para publicar la velocidad
  def control(self, linea):
    self.error = linea-self.setPoint
    #print("Error: ", self.error)
    self.vw.angular.z = -self.error*0.0007 #0.0005
    self.vw.linear.x = 0.14-abs(self.vw.angular.z) #0.12

  def pubVel(self):
    self.pubvelo.publish(self.vw)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert img to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Reduce img size
    #print('Original Dimensions : ', gray.shape)
    width = 500
    height = 400
    dim = (width, height)
    resized = cv2.resize(gray, dim)
    #print('New Dimensions : ',resized.shape)

    # Gaussian Blur
    preImage = cv2.GaussianBlur(resized,(5,5),0)

    # Kernel
    kernel = np.ones((5,5), np.uint8)

    # Erosion and dilation
    #img_erosion = cv2.erode(preImage, kernel, iterations=1)
    img_dilation = cv2.dilate(preImage, kernel, iterations=1)

    # Region of interest
    H = 40
    W = 300
    cropped_image = img_dilation[340:400, 100:400]
    #print(cropped_image.shape)	

    # Increase height
    #H2 = 3500
    #dim = (W, H2)
    #tall = cv2.resize(cropped_image, dim)
    #print('New tall Dimensions : ', tall.shape)

    #x = list(range(0,500))
    #dark = np.argmin(cropped_image, axis=0)

    aux = False
    sumaC = np.add.reduce(cropped_image, axis=0)
    sumaR = np.add.reduce(cropped_image, axis=1)
    linea = np.argmin(sumaC)
    lineaHor = np.argmin(sumaR)
    print("Linea Horizontal: ", lineaHor)
    print("Linea Vertical: ", linea) 

    ###################################################
    #Convert the BGR image to HSV colour space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #Set the lower and upper bounds for the green hue
    lower_green = np.array([60,60,150])
    upper_green = np.array([100,255,215])

    #Set the lower and upper bounds for the blue hue
    lower_blue = np.array([85,64,55])
    upper_blue = np.array([130,255,255])

    #Set the lower and upper bounds for the red hue
    lower_red1 = np.array([0,70,50])
    upper_red1 = np.array([10,255,255])
    lower_red2 = np.array([170,70,50])
    upper_red2 = np.array([180,255,255])

    #Create a mask for red and green colour using inRange function
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.add(red_mask1,red_mask2)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    #Perform bitwise and on the original image arrays using the mask
    res_red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    res_green = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
    res_blue = cv2.bitwise_and(cv_image, cv_image, mask=blue_mask)

    #Invert black and white
    invertR = cv2.bitwise_not(red_mask)
    ret, thR = cv2.threshold(invertR, 120, 255, cv2.THRESH_TOZERO)
    invertG = cv2.bitwise_not(green_mask)
    ret, thG = cv2.threshold(invertG, 120, 255, cv2.THRESH_TOZERO)
    invertB = cv2.bitwise_not(blue_mask)
    ret, thB = cv2.threshold(invertB, 120, 255, cv2.THRESH_TOZERO)
   
    ########################################################################################################v
    #Eliminate small blobs
    r_contours, r_hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    #g_contours, g_hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    b_contours, b_hierarchy = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # Maximum area for accepted blobs
    threshold_blob_area = 5000

    # Red Noise blobs
    for i in range(1, len(r_contours)):
      index_level = int(r_hierarchy[0][i][1])
      if index_level <= i:
        cnt = r_contours[i]
        area = cv2.contourArea(cnt)
        #print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(red_mask, [cnt], -1, 0, -1, 1)

    # Blue Noise blobs
    for i in range(1, len(b_contours)):
      index_level = int(b_hierarchy[0][i][1])
      if index_level <= i:
        cnt = b_contours[i]
        area = cv2.contourArea(cnt)
        #print(area)
      if(area) <= threshold_blob_area:
        cv2.drawContours(blue_mask, [cnt], -1, 0, -1, 1)

    #Find contour of a blob
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    opening_red = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=4)
    opening_green = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=4)

    #Get coordinates
    #Red rectangle 
    xR,yR,wR,hR = cv2.boundingRect(opening_red)
    cv2.rectangle(opening_red, (xR, yR), (xR + wR, yR + hR), (139,0,0), 4)
    xG,yG,wG,hG = cv2.boundingRect(opening_green)
    cv2.rectangle(opening_green, (xG, yG), (xG + wG, yG + hG), (139,0,0), 4)
    xB,yB,wB,hB = cv2.boundingRect(opening_green)
    cv2.rectangle(opening_green, (xB, yB), (xB + wB, yB + hB), (139,0,0), 4)
    
    #Notify when a color circle is found
    if(wR > 50) and (wR<500):
      self.msgs_motores.publish("Rojo")
      self.semafo = "Rojo"
      print(self.semafo)

    elif(wG > 40) and (wG<500):
      self.msgs_motores.publish("Verde")
      self.semafo = "Verde"
      print(self.semafo)

    elif(wB > 5) and (wB<500):
      self.msgs_motores.publish("Azul")
      azul = "Azul"
      self.auxB = True
      print(azul)
      
    else:
      self.msgs_motores.publish("X")
      self.semafo = "X"
      #print(self.semafo)
    ##########################################################################################

     #if((linea < 1 or linea >self.size-2) ):
         #aux2 = True
    aux2 = False
    print(self.aux)

    if(self.semafo == "Rojo" and (lineaHor != 59 or linea < 1 or linea >self.size-2)):
        self.vw.linear.x = 0.0
	self.vw.angular.z = 0.0
        self.auxR = True 
        print("Detenerse")

    # Linea Recta
    elif(self.semafo == "Verde") and (lineaHor != 59 or linea < 1 or linea >self.size-2):
        if(self.aux == False):
	  self.vw.linear.x = 0.07
	  self.vw.angular.z = 0.0
          self.pubVel()
	  rospy.sleep(4)
	  #aux2 = True
	  self.aux = True
	  print("Aux",self.aux)
	  print("Recto")
        else:
	  self.vw.linear.x = 0.18
 	  self.vw.angular.z = -0.15
 	  rospy.sleep(0.19)
	  print("Vuelta")

    elif(self.semafo == "Verde" and (linea > 1 or linea <self.size-2)):
	self.control(linea)

    elif(self.semafo == "Rojo" and (linea > 1 or linea <self.size-2)):
	self.control(linea)

    elif(self.semafo == "X" and (linea > 1 or linea <self.size-2)):
	self.control(linea)
        #print("Aux",self.aux)
    self.pubVel()

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  print("hola")
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  # Exit
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)

#!/usr/bin/env python
# Reto final: Puzzlebot
# Equipo 3
# Noemi Carolina Guerra Montiel A00826944
# Maria Fernanda Hernandez Montes A01704918
# Mizael Beltran Romero A01114973
# Izac Saul Salazar Flores A01197392
# Deteccion de linea, vision de semaforos y planeacion de ruta

# Librerias
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int32, Int32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

# Clase para acceder a la camara
class image_converter:

  def __init__(self):
    # Publicar y suscribirse a topicos
    self.image_pub = rospy.Publisher("Video",Image, queue_size = 10)
    self.pubvelo = rospy.Publisher("cmd_vel", Twist, queue_size=10) #Topic de cmd_vel para publicar las velocidades
    self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)
	
    self.bridge = CvBridge()
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
    self.k = 0.0007
    self.l = 0.13
    self.aux = False
 
  # Control para el seguimiento de linea
  def control(self, linea):
    self.error = linea-self.setPoint
    #print("Error: ", self.error)
    self.vw.angular.z = -self.error*self.k
    self.vw.linear.x = self.l-abs(self.vw.angular.z) 
	
  # Publicar la velocidad
  def pubVel(self):
    self.pubvelo.publish(self.vw)

  # Callback del topico de la camara
  def callback(self,data):
    # Obtener la imagen con el bridge
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ########## LINEA  #####################################

    # Convierte la imagen a escala de grises
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Reduce tamano de la imagen
    #print('Original Dimensions : ', gray.shape)
    width = 500
    height = 400
    dim = (width, height)
    resized = cv2.resize(gray, dim)
    #print('New Dimensions : ',resized.shape)

    # Gaussian Blur
    preImage = cv2.GaussianBlur(resized,(5,5),0)

    kernel = np.ones((5,5), np.uint8)
    # Dilation
    img_dilation = cv2.dilate(preImage, kernel, iterations=1)

    # Region de interes
    H = 60
    W = 300
    cropped_image = img_dilation[340:400, 100:400]
    #print(cropped_image.shape)	

    # Matriz de pixeles
    # Suma Vertical
    sumaC = np.add.reduce(cropped_image, axis=0)
    # Suma Horizontal
    sumaR = np.add.reduce(cropped_image, axis=1)
    # Encontrar los minimos
    linea = np.argmin(sumaC)
    lineaHor = np.argmin(sumaR)
    print("Linea Horizontal: ", lineaHor)
    print("Linea Vertical: ", linea) 

    ########  SEMAFOROS  ###########################################
    #Convert the BGR image to HSV colour space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #Set the lower and upper bounds for the green hue
    lower_green1 = np.array([45,90,90])
    upper_green1 = np.array([75,255,255])
    lower_green2 = np.array([36,52,72])
    upper_green2 = np.array([86,255,255])

    #Set the lower and upper bounds for the red hue
    lower_red1 = np.array([0,70,50])
    upper_red1 = np.array([10,255,255])
    lower_red2 = np.array([170,70,50])
    upper_red2 = np.array([180,255,255])

    #Create a mask for red and green colour using inRange function
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.add(red_mask1,red_mask2)
    green_mask1 = cv2.inRange(hsv, lower_green1, upper_green1)
    green_mask2 = cv2.inRange(hsv, lower_green2, upper_green2)
    green_mask = cv2.add(green_mask1,green_mask2)

    #Perform bitwise and on the original image arrays using the mask
    res_red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    res_green = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

    #Invert black and white
    invertR = cv2.bitwise_not(red_mask)
    ret, thR = cv2.threshold(invertR, 120, 255, cv2.THRESH_TOZERO)
    invertG = cv2.bitwise_not(green_mask)
    ret, thG = cv2.threshold(invertG, 120, 255, cv2.THRESH_TOZERO)
   
    #Eliminate small blobs
    r_contours, r_hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
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
    
    #Notify when a color circle is found
    if(wR > 5) and (wR<500):
      self.semafo = "Rojo"
      print(self.semafo)

    elif(wG > 100) and (wG<500):
      self.semafo = "Verde"
      print(self.semafo)
      
    else:
      self.semafo = "X"
      #print(self.semafo)
	
    ################ LOGICA #####################################

    if(self.semafo == "Rojo" and (lineaHor != 59 or linea < 1 or linea >self.size-2)):
        self.vw.linear.x = 0.0
	self.vw.angular.z = 0.0
	print("Detenerse")

     # Linea Recta
    elif((self.semafo == "Verde") and (linea < 1 or linea >self.size-2) and self.aux == False):
	self.vw.linear.x = 0.1
	self.vw.angular.z = 0.0
        self.pubVel()
	rospy.sleep(6)
	#aux2 = True
	self.aux = True
        print(self.aux)
	print("Recto")
	
    # Vuelta
    elif(self.semafo == "Verde" and (linea < 1 or linea >self.size-2) and self.aux == True):
	self.vw.linear.x = 0.19
 	self.vw.angular.z = -0.15
 	rospy.sleep(0.19)
	print("Vuelta")

    elif(self.semafo == "Verde" and (linea > 1 or linea <self.size-2)):
	self.control(linea)

    elif(self.semafo == "Rojo" and (linea > 1 or linea <self.size-2)):
	self.control(linea)

    elif(self.semafo == "X" and (linea > 1 or linea <self.size-2)):
	self.control(linea)
	
    self.pubVel()
    

    #Display the images
    #cv2.imshow("Original", cv_image)
    #cv2.imshow("Res green", res_green)
    #cv2.imshow("Res red", res_red)
    #cv2.imshow("Red Mask", red_mask)
    #cv2.imshow("Green Mask", green_mask)  
    #cv2.imshow("Opening Red", opening_red)
    #cv2.imshow("Opening Yellow", opening_yellow)
    #cv2.imshow("cropped", cropped_image)
    #cv2.imshow("linea", linea)
    #plt.show(pixel_plot)
    #cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  print("Bienvenido")
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

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

    #Funcion para publicar la velocidad
  def pubVel(self):
    #Publicamos la velocidad
    self.pubvelo.publish(self.vw)

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
    lower_red1 = np.array([0,100,20])
    upper_red1 = np.array([8,255,255])
    lower_red2 = np.array([175,100,20])
    upper_red2 = np.array([179,255,255])

    #Set the lower and upper bounds for the yellow hue
    lower_yellow = np.array([20,100,120]) #Azul (20, 90, 90)
    upper_yellow = np.array([40,255,255]) #Azul (255, 255, 255)

    #Create a mask for red, yellow and green colour using inRange function
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.add(red_mask1,red_mask2)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #Perform bitwise and on the original image arrays using the mask
    res_red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    res_green = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
    res_yellow=cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)

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
    y_contours, y_hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
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
    opening_green = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=4)
    opening_yellow = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=4)

    #Get coordinates
    #Red rectangle 
    xR,yR,wR,hR = cv2.boundingRect(opening_red)
    cv2.rectangle(opening_red, (xR, yR), (xR + wR, yR + hR), (139,0,0), 4)
    xY,yY,wY,hY = cv2.boundingRect(opening_yellow)
    cv2.rectangle(opening_yellow, (xY, yY), (xY + wY, yY + hY), (139,0,0), 4)
    
    #Notify when a color circle is found
    if(wR > 5) and (wR<500):
      print("Encontre un circulo rojo")
      self.msgs_motores.publish("Rojo")
      list = Int32MultiArray()
      #list.data = [xR, wR, 0]
      #self.msgs_seguidor.publish(list)
    elif(wY > 5) and (wY<500):
      print("Encontre un circulo amarillo")
      self.msgs_motores.publish("Amarillo")
      list = Int32MultiArray()
      list.data = [xY, wY, 0]
      self.msgs_seguidor.publish(list)
    else:
      self.msgs_motores.publish("Rojo")
      print("No hay circulos")

    ##########################################################################################
    # Convert img to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Reduce img size
    print('Original Dimensions : ', gray.shape)
    width = 500
    height = 400
    dim = (width, height)
    resized = cv2.resize(gray, dim)
    print('New Dimensions : ',resized.shape)

    # Gaussian Blur
    preImage = cv2.GaussianBlur(resized,(5,5),0)

    # Kernel
    kernel = np.ones((5,5), np.uint8)

    # Erosion and dilation
    img_erosion = cv2.erode(preImage, kernel, iterations=1)
    img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)

    # Region of interest
    W = 500
    H = 400
    cropped_image = img_dilation[300:400, 100:400]
    print(cropped_image.shape)	

    # Increase height
    #H2 = 3500
    #dim = (W, H2)
    #tall = cv2.resize(cropped_image, dim)
    #print('New tall Dimensions : ', tall.shape)

    #x = list(range(0,500))
    dark = np.argmin(cropped_image, axis=0)

    sumaC = np.add.reduce(cropped_image, axis=0)
    linea = np.argmin(sumaC)
    print(linea) 

    if(linea > (H/2)-80 and linea < (H/2)+80):
      self.vw.linear.x = 0.1
      self.vw.angular.z = 0
      print("Linea centro") 

    elif(linea < 10):
      self.vw.linear.x = 0
      self.vw.angular.z = 0.05
      print("Linea derecha arreglada 0") 

    elif(linea == 299):
      self.vw.linear.x = 0
      self.vw.angular.z = 0.05
      print("Linea derecha arreglada 300")

    elif(linea < (H/2)-80): 
      self.vw.linear.x = 0
      self.vw.angular.z = 0.05
      print("Linea derecha") 
 
    elif(linea > (H/2)+80 and linea < H-10):
      self.vw.linear.x = 0
      self.vw.angular.z = -0.05
      print("Linea izquierda")

    else: 
      self.vw.linear.x = 0
      self.vw.angular.z = 0 
      print("No encontre la linea")

    self.pubVel()
    #Graph
    #pixel_plot = plt.figure()
    #plt.plot(x,dark) 
    #plt.xlim([-10,W])
    #plt.ylim([-10,H2+100])

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

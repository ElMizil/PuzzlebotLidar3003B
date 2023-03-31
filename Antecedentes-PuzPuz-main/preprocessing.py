#!/usr/bin/env python
# Actividad 4 Preprocesamiento de imagen obtenida con camara del puzzlebot.
# Convertir de imagen de ROS a imagen de OpenCv y viceversa

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

''' 
# Access image in ROS
bridge = cv_bridge.CvBridge() # Transmit images from ROS to opencv
img = bridge.imgmsg_to_cv2(msg, encoding="passthrough") # Convert an incoming ros msg from a sub to opencv format
img_back = bridge.cv2_to_imgmsg(result_image)# Convert from OpenCV format to ROS msg format
'''

''' CODIGO EN OPENCV SIN ROS 
path = r'/home/noemi/puzzlebot/vision/src/traffic_light/imgs/road.jpeg'
src = cv2.imread(path, cv2.IMREAD_UNCHANGED) # Read img

# Rotate img 180
img = cv2.rotate(src, cv2.ROTATE_180)

# Convert img to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Reduce img size
print('Original Dimensions : ', gray.shape)
scale_percent = 60 # percent of original size
width = int(gray.shape[1] * scale_percent / 100)
height = int(gray.shape[0] * scale_percent / 100)
dim = (width, height)
resized = cv2.resize(gray, dim)
print('Original Dimensions : ',resized.shape)

# Gaussian Blur
result_image = cv2.GaussianBlur(resized,(5,5),0)

# Display img
cv2.imshow('Pre-processed image', result_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("video_source/raw", Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert img to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Reduce img size
    print('Original Dimensions : ', gray.shape)
    scale_percent = 60 # percent of original size
    width = int(gray.shape[1] * scale_percent / 100)
    height = int(gray.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(gray, dim)
    print('Original Dimensions : ',resized.shape)

    # Gaussian Blur
    result_image = cv2.GaussianBlur(resized,(5,5),0)

    # Display img
    cv2.imshow('Pre-processed image', result_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)

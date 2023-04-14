#!/usr/bin/env python2

import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

print("Waiting for 5 seconds.")
time.sleep(5)
print("Wait is over.")
t=rospy.get_rostime() #get time as rospy.Time instance
t2=rospy.get_time() #get time as float secs
print(t)
print(t2)

'''
class part2:
    def __init__(self):
        #Iniciamos los suscriptores y publicadores
        rospy.Subscriber('/pose_sim',PoseStamped,self.setPoseF)
        self.wr = rospy.Publisher('/wr', Float32, queue_size=10)
        self.wl = rospy.Publisher('/wl', Float32, queue_size=10)

    def setPoseF(self,msg):
        self.vel_angular = msg.angular.z
        self.vel_linear = msg.linear.x

'''
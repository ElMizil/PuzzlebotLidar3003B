#!/usr/bin/env python2
# Equipo 3
# Simulador cinematico con incertidumbre para Puzzlebot con RVIZ
import rospy
import tf_conversions
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Vector3
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
import math
import numpy as np

#Declaramos la clase del puzzlebot
class puzzlebot_cinematic:
  def __init__(self):
     #Iniciamos las velocidades y la posicion del robot
     self.vel_linear=0
     self.vel_angular=0
     self.x = 0
     self.y = 0
     self.angle = 0
     #Iniciamos datos necesarios como el radio de las llantas y la frecuencia
     self.r = 0.2
     self.freq = 150
     #Iniciamos los suscribers y publishers
     rospy.Subscriber('/cmd_vel',Twist,self.setVel)
     self.pose_pub = rospy.Publisher('/pose_sim', PoseStamped, queue_size=10)
     self.wr = rospy.Publisher('/wr', Float32, queue_size=10)
     self.wl = rospy.Publisher('/wl', Float32, queue_size=10)
     self.path_pub = rospy.Publisher('/path',Path, queue_size=10)
     self.odom = rospy.Publisher('/odom',Odometry,queue_size=10)
     #Iniciamos variables de la odometria
     self.eK = np.array((0,0,0,
                0,0,0,
                0,0,0)).reshape(3,3)
     self.covariance=[0]*36


  #Metodo para asignar la velocidad obtenida de /cmd_vel
  #Nota: Solo necesitamos la velocidad lineal en x y la angular en z
  def setVel(self,msg):
     self.vel_angular = msg.angular.z
     self.vel_linear = msg.linear.x

  #Metodo para obtener la posicion y orientacion del robot
  def getPuzzleVel(self):
     self.x += self.vel_linear*math.cos(self.angle)/self.freq
     self.y += self.vel_linear*math.sin(self.angle)/self.freq
     self.angle += math.radians(self.vel_angular)
     self.getPuzzlePos()

  #Metodo para obtener la pose del robot para publicar en /pose_sim
  def getPuzzlePos(self):
     pose = PoseStamped()
     #Empezamos poniendo la posicion
     pose.pose.position.x = self.x
     pose.pose.position.y = self.y
     #Luego generamos los quaterniones
     q = quaternion_from_euler(0,0,self.angle)

     pose.pose.orientation.x = q[0]
     pose.pose.orientation.y = q[1]
     pose.pose.orientation.z = q[2]
     pose.pose.orientation.w = q[3]

     pose.header.frame_id = "base_link"
     #Publicamos el pose y mandamos la transformada
     self.broadcastTransform(pose.pose.orientation)
     self.pose_pub.publish(pose)
    

  #Mandamos la transformada usando un broadcaster
  def broadcastTransform(self, orientation):
       #Iniciamos el broadcaster
       br = TransformBroadcaster()
       t = TransformStamped()
       #Llenamos la transformada con la posicion y las orientaciones
       t.header.stamp = rospy.Time.now()
       #Nombres de los frames
       t.header.frame_id = "base_link"
       t.child_frame_id = "chassis"
       t.transform.translation = Vector3(self.x, self.y, 0)
       t.transform.rotation = orientation
       #Mandamos la transformada
       br.sendTransform(t)
       #Generamos el camino
       m = Path()
       m.header = t.header
       pose = PoseStamped()
       pose.header = t.header
       pose.pose.position.x = t.transform.translation.x
       pose.pose.position.y = t.transform.translation.y
       pose.pose.position.z = t.transform.translation.z
       pose.pose.orientation = t.transform.rotation
       m.poses.append(pose)
       self.path_pub.publish(m)
       #Generamos la odometria
       odometry = Odometry()
       odometry.header = t.header
       odometry.child_frame_id = t.child_frame_id
       odometry.pose.pose = pose.pose
       odometry.pose.covariance = self.covariance
       odometry.twist.twist.linear.x = self.vel_linear
       odometry.twist.twist.linear.y = 0
       odometry.twist.twist.linear.z = 0
       odometry.twist.twist.angular.x = 0
       odometry.twist.twist.angular.y = 0
       odometry.twist.twist.angular.z = self.vel_angular
       odometry.twist.covariance = self.covariance
       self.getCovariance()
       self.odom.publish(odometry)
 

  #Metodo para obtener la matrix de covarianza
  def getCovariance(self):
     #Iniciamos las matrices que necesitamos con los valores obtenidos
     H = np.matrix([[1, 0, -self.vel_linear*math.sin(self.angle)/self.freq],
          [0, 1, self.vel_linear*math.cos(self.angle)/self.freq],
          [0, 0, 1]])
     Ht = H.transpose()

     wk = self.r/(2*self.freq)*np.matrix([[math.cos(self.angle),math.cos(self.angle)],
                                          [math.sin(self.angle),math.sin(self.angle)],
                                          [2/self.l, -2/self.l]])
     wkt = wk.transpose()

     Edk = np.matrix([[0.8*abs((self.vel_linear + self.vel_angular)/self.r), 0],
                     [0, 0.8*abs((self.vel_linear - self.vel_angular)/self.r)]])
    
     Qk = wk*Edk*wkt
     #Obtenemos la matrix de covarianza 3x3
     self.eK = np.array(H*self.eK*Ht+Qk).reshape(3,3)
     #Rellenamos los 9 datos que tenemos y el resto con 0
     self.covariance=[self.eK[0][0],self.eK[0][1],0,0,0,self.eK[0][2],self.eK[1][0],self.eK[1][1],0,0,0,self.eK[1][2],0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,self.eK[2][0],self.eK[2][1],0,0,0,self.eK[2][2]]
 
  #Metodo para obtener las velocidades de las llantas
  def getWheelVel(self):
     #Calculamos la velocidad de cada rueda y lo publicamos
     self.wl.publish((self.vel_linear - self.vel_angular)/self.r)
     self.wr.publish((self.vel_linear + self.vel_angular)/self.r)

if __name__ == '__main__':
  rospy.init_node('puzzlebot_sim')
  puzzlebot = puzzlebot_cinematic()
  rate = rospy.Rate(150)
  while not rospy.is_shutdown():
     puzzlebot.getPuzzleVel()
     puzzlebot.getWheelVel()
     rate.sleep()


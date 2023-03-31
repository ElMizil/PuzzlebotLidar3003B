#!/usr/bin/env python
#Codigo usado actualmente como demo para pruebas de control P (Ir a puntos dados por el usuario), Equipo 3
#Librerias que contienen funciones para el programa
from cmath import pi
from math import atan2, cos, sin, sqrt
import rospy, time
#Librerias de mensajes de ROS
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String


#Creamos una clase para el puzzlebot
class Puzzlebot:

    #Funcion para inicializar todas las variables de la clase
    def __init__(self):
        self.r = 0.05
        self.l = 0.19
        self.thetha = 0
        self.x = 0
        self.y = 0
        self.xt = 0.3
        self.yt = 0.3
        self.wl = 0
        self.wr = 0
        self.dt = 0.01
        self.vw = Twist()
        self.vw.linear.x=0
        self.vw.linear.y=0
        self.vw.linear.z=0
        self.vw.angular.x=0
        self.vw.angular.y=0
        self.vw.angular.z=0
        self.kv = 0.5
        self.kw = 0.3

    #Funcion de callback cuando el topico /wr mande un mensaje    
    def whr(self, msg):
        pubr.publish(msg.data)
        self.wr = msg.data

    #Funcion de callback cuando el topico /wl mande un mensaje
    def whl(self, msg):
        publ.publish(msg.data)
        self.wl = msg.data

    #Funcion para calcular la posicion actual del robot
    def position(self):
        self.thetha = self.thetha + self.r*((self.wr-self.wl)/self.l)*self.dt
        #Mantenemos el valor de theta en un intervalo de -pi a pi
        if(self.thetha > pi):
            self.thetha = self.thetha-pi
        if(self.thetha < -pi):
            self.thetha = self.thetha+pi

        self.x = self.x + self.r*((self.wr+self.wl)/2)*self.dt*cos(self.thetha)
        self.y = self.y + self.r*((self.wr+self.wl)/2)*self.dt*sin(self.thetha)    
        #Publicamos la posicion actual
        pubp.publish(self.x,self.y,self.thetha)
        #print("Posicion actualizada")

    #Funcion para calcular el error 
    def error(self):
        if(self.thetha >= 0):
            self.et=atan2(self.yt,self.xt)-self.thetha
        elif (self.thetha < 0) :
            self.et=-atan2(self.yt,self.xt)-self.thetha
        self.ed = sqrt(pow((self.xt-self.x),2)+pow((self.yt-self.y),2))
        #Si el error se hace muy pequeno lo volvemos 0
        #Publicamos el error
        pubed.publish(self.et)
        pubet.publish(self.ed)

    def calculaVel(self):
        self.vw.linear.x = self.ed*self.kv
        self.vw.angular.z = self.et*self.kw

    #Funcion para publicar la velocidad
    def pubVel(self):
        #Publicamos la velocidad
        pubvel.publish(self.vw)
if __name__ == '__main__':
    try:
        #Iniciamos el puzzlebot junto con todos los topicos que necesitamos
        puzzlebot = Puzzlebot()
        pubr = rospy.Publisher("vel_wr", Float32, queue_size=10) #Opcional
        publ = rospy.Publisher("vel_wl", Float32, queue_size=10) #Opcional
        pubp = rospy.Publisher("pos", Vector3, queue_size=10) #Topic de la posicion
        pubed = rospy.Publisher("errort",Float32, queue_size=10) #Topic del error del angulo
        pubet = rospy.Publisher("errord", Float32, queue_size=10) #Topic del error de la distancia
        pubvel = rospy.Publisher("cmd_vel", Twist, queue_size=10) #Topic de cmd_vel para publicar las velocidades
        rospy.init_node("control")
        #rate = rospy.Rate(100)
        rospy.Subscriber("wr", Float32, puzzlebot.whr)#Topic de wr para recibir la velocidad derecha
        rospy.Subscriber("wl", Float32, puzzlebot.whl)#Topic de wr para recibir la velocidad izquierda 
        #rospy.Subscriber("Mensaje_semaforo", String, puzzlebot.semaforo)#Topic del semaforo para recibir el color 
        base = rospy.get_time()
        now=rospy.get_time()
        desp=sqrt(pow(puzzlebot.xt,2)+pow(puzzlebot.yt,2))
        while not rospy.is_shutdown():
            now=rospy.get_time()
            puzzlebot.dt=now-base
            if puzzlebot.dt >= 0.01:
                puzzlebot.position()
                puzzlebot.error()#Los argumentos son la posicion deseada ej. (0.8,0) (esta en metros)
                puzzlebot.calculaVel()
                if puzzlebot.et > 0.03:
                    puzzlebot.vw.linear.x=0
                if puzzlebot.ed < desp*.05:
                    puzzlebot.vw.linear.x=0
                    puzzlebot.vw.angular.z=0
                pubp.publish(puzzlebot.x, puzzlebot.y, puzzlebot.thetha)    
                pubvel.publish(puzzlebot.vw)
                base=rospy.get_time()
    except rospy.ROSInterruptException:
        pass

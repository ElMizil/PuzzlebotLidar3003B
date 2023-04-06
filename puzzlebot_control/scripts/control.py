#!/usr/bin/env python3
#Librerias que contienen funciones para el programa
from cmath import pi
from math import atan2, cos, sin, sqrt
import rospy, time
#Librerias de mensajes de ROS
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32


#Creamos una clase para el puzzlebot
class Puzzlebot:

    #Funcion para inicializar todas las variables de la clase
    def __init__(self):
        #Caracteristicas del robot y todo lo necesario para el controlador PID
        self.r = 0.05
        self.l = 0.19
        self.theta = 0
        self.x = 0
        self.y = 0
        self.xt = 0.7
        self.yt = 0
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
        self.kv = 0.1
        self.kw = 0.3
        self.ki = 0.15
        self.kd = 0.1
        self.sum_ed = 0
        self.last_ed = 0
        self.ed = 0
        self.sum_et = 0
        self.last_et = 0
        self.et = 0

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
        self.theta = self.theta + self.r*((self.wr-self.wl)/self.l)*self.dt
        #Mantenemos el valor de theta en un intervalo de -pi a pi
        if(self.theta > pi):
            self.theta = self.theta-pi
        if(self.theta < -pi):
            self.theta = self.theta+pi

        self.x = self.x + self.r*((self.wr+self.wl)/2)*self.dt*cos(self.theta)
        self.y = self.y + self.r*((self.wr+self.wl)/2)*self.dt*sin(self.theta)    
        #Publicamos la posicion actual
        pubp.publish(self.x,self.y,self.theta)
        #print("Posicion actualizada")

    #Funcion para calcular el error 
    def error(self):
        #Calculamos el error en el angulo
        self.last_et = self.et
        if(self.theta >= 0):
            self.et=atan2(self.yt,self.xt)-self.theta
        elif (self.theta < 0) :
            self.et=-atan2(self.yt,self.xt)-self.theta
        self.sum_et += self.et*self.dt
        #Calculamos el error en la distancia
        self.last_ed = self.ed
        self.ed = sqrt(pow((self.xt-self.x),2)+pow((self.yt-self.y),2))
        self.sum_ed += self.ed*self.dt
        #Publicamos el error
        pubed.publish(self.et)
        pubet.publish(self.ed)

    #Funcion para calcular la velocidad
    def calculaVel(self):
        #Calculamos la velocidad con el PID
        #La P es multiplicar el error por la kp
        #La I es ir sumando los errores y multiplicar por ki
        #La D es restar el error actual por el viejo y multiplicar por kd
        self.vw.linear.x = self.ed*self.kv + self.sum_ed*self.ki + self.kd*((self.ed-self.last_ed)/self.dt)
        self.vw.angular.z = self.et*self.kw + self.sum_et*self.ki + self.kd*((self.et-self.last_et)/self.dt)


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
        rospy.Subscriber("wr", Float32, puzzlebot.whr)#Topic de wr para recibir la velocidad derecha
        rospy.Subscriber("wl", Float32, puzzlebot.whl)#Topic de wr para recibir la velocidad izquierda 
        base = rospy.get_time()
        now=rospy.get_time()
        #Calculamos el desplazamiento
        desp=sqrt(pow(puzzlebot.xt,2)+pow(puzzlebot.yt,2))
        while not rospy.is_shutdown():
            now=rospy.get_time()
            puzzlebot.dt=now-base
            if puzzlebot.dt >= 0.01:
                #Calculamos la posicion,error y velocidad del robot
                puzzlebot.position()
                puzzlebot.error()
                puzzlebot.calculaVel()
                if puzzlebot.et > 0.03:
                    puzzlebot.vw.linear.x=0
                #Si la el error en la distancia es pequeño detenemos el robot
                if puzzlebot.ed < desp*.02:
                    puzzlebot.vw.linear.x=0
                    puzzlebot.vw.angular.z=0
                    pubvel.publish(puzzlebot.vw)
                #Publicamos la posicion y velocidad actual del robot
                pubp.publish(puzzlebot.x, puzzlebot.y, puzzlebot.theta)    
                pubvel.publish(puzzlebot.vw)
                base=rospy.get_time()
    except rospy.ROSInterruptException:
        pass
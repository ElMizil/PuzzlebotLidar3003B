#!/usr/bin/env python3
#Librerias que contienen funciones para el programa
from cmath import pi
from math import atan2, cos, sin, sqrt
import rospy
#Librerias de los mensajes de ROS
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3


#Creamos una clase para el robot
class Robot:

    #Funcion para inicializar todas las variables de la clase
    def __init__(self):
        #Caracteristicas del robot
        self.r = 0.05
        self.l = 0.19
        #Posicion inicial
        self.x = 0
        self.y = 0
        self.thetha = 0
        #Punto objetivo
        self.xt = 1
        self.yt = 0
        #Velocidad inicial llantas
        self.wl = 0
        self.wr = 0
        self.dt = 0.01
        #Mensaje para manejar las velocidades del robot
        self.vel = Twist()
        self.vel.linear.x=0
        self.vel.linear.y=0
        self.vel.linear.z=0
        self.vel.angular.x=0
        self.vel.angular.y=0
        self.vel.angular.z=0
        #Parametros para el PID
        self.kp = 0.1
        self.ki = 0.15
        self.kd = 0.1
        self.ed = 0
        self.total_ed = 0
        self.ultimo_ed = 0
        self.et = 0
        self.total_et = 0
        self.ultimo_et = 0

        
    #Funcion de callback cuando el topico /wr mande un mensaje    
    def wr_callback(self, msg):
        self.wr = msg.data

    #Funcion de callback cuando el topico /wl mande un mensaje
    def wl_callback(self, msg):
        self.wl = msg.data

    #Funcion para calcular la posicion actual del robot
    def calculaPos(self):
        #Calculamos el angulo del robot
        self.thetha = self.thetha + self.r*((self.wr-self.wl)/self.l)*self.dt
        #Mantenemos el valor de theta en un intervalo de -pi a pi
        if(self.thetha > pi):
            self.thetha = self.thetha-pi
        if(self.thetha < -pi):
            self.thetha = self.thetha+pi
        #Calculamos la posicion x y
        self.x = self.x + self.r*((self.wr+self.wl)/2)*self.dt*cos(self.thetha)
        self.y = self.y + self.r*((self.wr+self.wl)/2)*self.dt*sin(self.thetha)    
        #Publicamos la posicion actual
        pos_pub.publish(self.x,self.y,self.thetha)

    #Funcion para calcular el error 
    def calculaError(self):
        #Guardamos el ultimo error para la D
        self.ultimo_ed = self.ed
        self.ultimo_et = self.et
        #Calculamos el error en la distancia y el angulo
        if(self.thetha >= 0):
            self.et=atan2(self.yt,self.xt)-self.thetha
        elif (self.thetha < 0):
            self.et=-atan2(self.yt,self.xt)-self.thetha
        self.ed = sqrt(pow((self.xt-self.x),2)+pow((self.yt-self.y),2))
        #Vamos sumando los errores para la I
        self.total_ed += self.ed*self.dt
        self.total_et += self.et*self.dt
        #Publicamos el error
        ed_pub.publish(self.et)
        et_pub.publish(self.ed)

    #Funcion para calcular la velocidad
    def calculaVel(self):
        #Calculamos la velocidad con el PID
        #La P es multiplicar el error por la kp
        #La I es ir sumando los errores y multiplicar por ki
        #La D es restar el error actual por el viejo y multiplicar por kd
        self.vel.linear.x = self.ed*self.kp + self.total_ed*self.ki + self.kd*((self.ed-self.ultimo_ed)/self.dt)
        self.vel.angular.z = self.et*self.kp + self.total_et*self.ki + self.kd*((self.et-self.ultimo_et)/self.dt)


if __name__ == '__main__':
    try:
        #Iniciamos el robot junto con todos los topicos que necesitamos
        robot = Robot()
        pos_pub = rospy.Publisher("pos", Vector3, queue_size=10) #Topic de la posicion
        ed_pub = rospy.Publisher("error_theta",Float32, queue_size=10) #Topic del error del angulo
        et_pub = rospy.Publisher("error_distancia", Float32, queue_size=10) #Topic del error de la distancia
        vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) #Topic para publicar las velocidades lineales y angulares
        rospy.init_node("PID")
        rospy.Subscriber("wr", Float32, robot.wr_callback)#Topic de wr para recibir la velocidad derecha
        rospy.Subscriber("wl", Float32, robot.wl_callback)#Topic de wr para recibir la velocidad izquierda 
        #Calculamos la distancia al destino
        destino = sqrt(pow(robot.xt,2)+pow(robot.yt,2))
        inicio = rospy.get_time()
        now =rospy.get_time() 
        while not rospy.is_shutdown():
            now=rospy.get_time()
            #Calculamos el dt
            robot.dt=now-inicio
            if robot.dt >= 0.01:
                #Calculamos la posicion actual 
                robot.calculaPos()
                #Calculamos el error actual
                robot.calculaError()
                #Calculamos la velocidad 
                robot.calculaVel()
                #Si hay error en theta detenemos el robot para que gire
                if robot.et > 0.03:
                    robot.vel.linear.x=0
                #Si el error ya es pequeño detenemos el robot
                if robot.ed < destino*.05:
                    robot.vel.linear.x=0
                    robot.vel.angular.z=0
                #Publicamos la posicion y velocidad actual del robot
                pos_pub.publish(robot.x, robot.y, robot.thetha)    
                vel_pub.publish(robot.vel)
                inicio=rospy.get_time()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python2
# Control de lazo abierto para repetir el experimento n veces durante m segundos y obtener la pose del robot en rviz y gazebo

# Librerias
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty
import xlsxwriter

# Obtener las posiciones finales y exportarlas a un excel
class chlg1_part2:
    def __init__(self):
      self.pos_rviz = PoseStamped()
      self.pos_gazebo = ModelState()
      self.yaw_gazebo = 0
      # Definir archivo
      self.workbook = xlsxwriter.Workbook('resultadosChlg1P2.xlsx')
      # Anadir hoja de calculo
      self.worksheet = self.workbook.add_worksheet()
      # Publishers y subscribers
      rospy.Subscriber('/pose_sim', PoseStamped, self.getPosRviz)
      rospy.Subscriber('/gazebo/model_states', ModelStates, self.getPosGazebo)
      self.reset_rviz = rospy.Publisher('/reset', Empty, queue_size=10)
      self.reset_gazebo = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
      self.vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Reiniciar el robot en su posicion inicial
    def reset(self):
       # Rviz
       self.reset_rviz.publish(Empty())
       # Gazebo
       ms = ModelState()
       ms.model_name = 'puzzlebot'
       ms.pose.position.x = 0
       ms.pose.position.y = 0
       ms.pose.orientation.w = 0
       self.reset_gazebo.publish(ms)

    # Obtener la posicion del robot en Rviz
    def getPosRviz(self, poseR):
       self.pos_rviz = poseR

    # Obtener la posicion del robot en Gazebo
    def getPosGazebo(self, poseG):
       self.pos_gazebo.pose = poseG.pose[poseG.name.index('puzzlebot')]

    # Imprimir valores en la terminal
    def showPos(self):
       # Positions
       print("x_rviz: ", self.pos_rviz.pose.position.x)
       print("y_rviz: ", self.pos_rviz.pose.position.y)
       print("x_gazebo: ", self.pos_gazebo.pose.position.x)
       print("y_gazebo: ", self.pos_gazebo.pose.position.y)
       # Orientation
       euler_angle_rviz = euler_from_quaternion([self.pos_rviz.pose.orientation.x, self.pos_rviz.pose.orientation.y, self.pos_rviz.pose.orientation.z, self.pos_rviz.pose.orientation.w])
       yaw_rviz = euler_angle_rviz[2]
       print("angle_rviz: ", yaw_rviz)
       euler_angle_gazebo = euler_from_quaternion([self.pos_gazebo.pose.orientation.x, self.pos_gazebo.pose.orientation.y, self.pos_gazebo.pose.orientation.z, self.pos_gazebo.pose.orientation.w])
       self.yaw_gazebo = euler_angle_gazebo[2]
       print("angle_gazebo: ", self.yaw_gazebo)
       print("--------------------------------------------")

    # Pasar los resultados a un archivo de excel
    def passResults(self, i):
        i = i+1
        # Escribir en la hoja de calculo
        self.worksheet.write('A%f' %i, self.pos_gazebo.pose.position.x)
        self.worksheet.write('B%f' %i, self.pos_gazebo.pose.position.y)
        self.worksheet.write('C%f' %i, self.yaw_gazebo)
        

if __name__ == '__main__':
    rospy.init_node('chlg1_part2')
    tm = chlg1_part2()
    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        t = 5 # Esperar 5 seg
        vStop = Twist()
        vStop.linear.x = 0
        vStop.linear.y = 0
        vStop.linear.z = 0
        vStop.angular.x = 0
        vStop.angular.y = 0
        vStop.angular.z = 0
        # Movimiento lineal
        vIni = Twist()
        vIni.linear.x = 0.2
        for i in range(21):
            tm.vel.publish(vIni)
            rospy.sleep(t) 
            print("LINEAR_VEL REP: ", i)
            tm.showPos()
            tm.passResults(i)
            tm.vel.publish(vStop)
            tm.reset()
        # Movimiento angular
        vIni.angular.z = 0.2
        for j in range(20):
            tm.vel.publish(vIni)
            rospy.sleep(t)
            print("ANGULAR_VEL REP: ", j+1)
            tm.showPos()
            tm.passResults(j+i+1)
            tm.vel.publish(vStop)
            tm.reset()
        tm.workbook.close() # Cerrar archivo
        print("Done!")
        rospy.spin()
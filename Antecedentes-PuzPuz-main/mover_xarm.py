#!/usr/bin/env python
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import tf2_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
#from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
from tf import TransformListener
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray

class Planner():
  def __init__(self):
    #Inicializamos la interfaz de moveit
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_py", anonymous=True)
    
    #Creamos el moveit commander del xarm y la escena
    xarm = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    #Creamos los move_groups del xarm y del gripper 
    group_name = "xarm6"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #group_name_gripper = "xarm_gripper"
    #gripper_move_group = moveit_commander.MoveGroupCommander(group_name_gripper)
    
    #Creamos un publisher para el topico para desplegar la trayectoria
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    
    #Inicializamos algunas variables del Planner
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = xarm.get_group_names()
    self.box_name = ""
    self.robot = xarm
    self.scene = scene
    self.move_group = move_group
    #self.gripper_move_group = gripper_move_group 
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    
  #Funcion para agregar las cajas y los depositos  
  def addObstacles(self):
    #Inicializamos algunas variables de posicion que nos serviran mas tarde
    pos = [10,15,5]
    posCajaVerde=[-0.186303,0.411113,1.045000]
    posCajaRoja = [-0.516300, 0.4, 1.044900]
    posCajaAzul = [-0.560639, 0.275921, 1.045000]
    posXarm=[-0.000001,0.200001,1.021000]
    path=[0.1,0.1,0.25]

    self.posCajaVerde = posCajaVerde
    self.posCajaRoja = posCajaRoja
    self.posCajaAzul = posCajaAzul
    self.posXarm = posXarm
    self.pos = pos
    self.path = path
  
  #Funcion para ir a la posicion deseada
  def goToPose(self,goal):
    move_group = self.move_group
    #posCajas = [self.posCajaVerde, self.posCajaRoja, self.posCajaAzul]
    posXarm = self.posXarm
    pos = self.pos
    path = self.path
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    
    #Creamos las posiciones apartir de las posiciones de la caja y el brazo
    #Nota: Recordar que "y" es "x" y viceversa por que en moveit y gazebo es diferente los ejes
    path = [float(goal[0]),-float(goal[1]),float(goal[2])]
    waypoints = [1]
    pos[0]=path[0]
    pos[1]=path[1]
    pos[2]=path[2]
    #Creamos un arreglo con las posiciones actuales del robot que luego cambiamos por las del objetivo
    #Nota: Se suma a z por que es necesario que el robot quede mas arriba de la caja sino chocaria con ella
    wpose = move_group.get_current_pose().pose
    wpose.position.z = pos[2]
    wpose.position.y = pos[1]
    wpose.position.x = pos[0]
    waypoints[0]=copy.deepcopy(wpose)
    #Usamos cartesian path para obtener la trayectoria hacia los puntos deseados
    (plan, fraction) = move_group.compute_cartesian_path(
    waypoints,   # waypoints a seguir
    0.01,        # eef_step
    0.0)         # jump_threshold
    # Ahora planeamos la trayectoria 
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publicamos la trayectoria y la ejecutamos
    display_trajectory_publisher.publish(display_trajectory)
    move_group.execute(plan, wait=True)
    rospy.sleep(0.5)
    
  

  def printPose(self):
    move_group = self.move_group
    wpose = move_group.get_current_pose().pose
    print(wpose.position.x)
    print("x \n")
    print(wpose.position.y)
    print("y \n")
    print(wpose.position.z)
    print("z \n")


  def callback(self, data):
    print("Coordenadas recibidas")
    goal = [0, 0, 0]
    #if(data[0] > 10):
    goal[0] = goal[0] + 0.4
    #if(data[1] > 10):
    goal[2] = goal[2] + 0.5
    self.goToPose(goal)

  


class myNode():
  def __init__(self):
    print("Nodo iniciado")
    
  def main(self):
    #Creamos nuestro Planner y agregamos los obstaculos(cajas y depositos)
    self.planner = Planner()
    self.planner.addObstacles()
    rospy.Subscriber("PosXarm", Float32MultiArray, self.planner.callback)
    #Hacemos un loop para repetir el proceso de agarrar el cubo y ponerlo en la caja
    #for i in (1,2,3):
        #Obtenemos nuestro objetivo de pick y obtenemos sus coordenadas con tf
        #box = self.getGoal("pick")
        #goal = self.tf_goal(box)
        #Vamos hacia la posicion del objetivo y lo agarramos y esperamos a que se registre el movimiento
    #goal = [0.5, 0, 0.112] #HOME: 0.207 0 0.112 
    #self.planner.goToPose(goal)
    while(True):
      self.planner.printPose()
      rospy.spin()
    #Apagamos el programa
    rospy.signal_shutdown("Task Completed")

if __name__ == '__main__':
  try:
    #Iniciamos nuestro Nodo y corremos el programa
    node = myNode()
    node.main()
  except rospy.ROSInterruptException:
    pass

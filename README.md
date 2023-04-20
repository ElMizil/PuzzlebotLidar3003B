# PuzzlebotLidar3003B
Repositorio de Puzzlebot con un Lidar
# Correr Proyecto
```
roslaunch manchester_week1 puzzlebot.launch
```
# Reto Semana 1
Diseño de modelo cinematico para el puzzlebot asi como su simulacion en RVIZ y Gazebo
### Diagrama de flujo
```mermaid
flowchart TD
    A(Start) -->Z[reps=0] 
    Z --> B[InitialPos=0, time=0]
    B -->|Get cmd_vel| C[Set wheels vel]
    C -->D[Calculate pose and transforms]
    D -->F{time==5s?}
    F -->|No| D
    F -->|Yes| H
    H[Print pose]
    H -->I{reps==20?}
    I--> |No| B
    I --> |Yes| J
    J(End)
```
```
roslaunch manchester_week1 puzzlebot.launch py:=puzzlebot.py
```
# Reto Semana 2
```
roslaunch manchester_week1 puzzlebot.launch py:=odometry.py
```
# Reto Semana 3
# Reto Semana 4
# Reto Semana 5

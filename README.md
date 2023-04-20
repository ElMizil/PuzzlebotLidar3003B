# PuzzlebotLidar3003B
Repositorio de Puzzlebot con un Lidar

# Reto Semana 1
Diseño de modelo cinematico para el puzzlebot asi como su simulacion en RVIZ y Gazebo
## Correr el programa
### Simulacion en Rviz
```
roslaunch puzzlebot_rviz puzzlebot_rviz.launch
```
### Simulacion en Gazebo
```
roslaunch puzzlebot_gazebo puzzlebot_gazebo.launch
```
### Nodo parte 1
```
rosrun puzzlebot_sim puzzlebotkinematic_model.py
```
*Incluido en los archivos launch*

### Nodo parte 2
```
rosrun puzzlebot_sim time_control.py
```
*NO incluido en los archivos launch*

## Diagrama de flujo
```mermaid
flowchart LR
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


# Reto Semana 2
# Reto Semana 3
# Reto Semana 4
# Reto Semana 5

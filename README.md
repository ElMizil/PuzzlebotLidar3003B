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
## Diagrama de flujo
```mermaid
flowchart LR
    A(Start) -->|Get cmd_vel| B[Set wheels vel]
    B -->C[Calculate pose, odom and transforms]
    C -->D[Calculate covariance matrix] 
    D -->|Print pose and odom| E(End)
```

# Reto Semana 4
BUG0
```mermaid
flowchart TD
    A(Inicio) -->B[Calcular camino a la meta]
    B -->C{Llego a la meta?}
    C -->|Si| Z[Fin] 
    C -->|No| D[Ir hacia la meta]
    D --> E{Obstaculo?}
    E -->|No| C
    E -->|Si| F[Seguir pared]
    F -->G{Puede seguir a la meta?} 
    G -->|Si| D
    G -->|No| F
```
   
BUG2
```mermaid
flowchart TD
    A(Inicio) -->B[Calcular linea-m]
    B -->C{Llego a la meta?}
    C -->|Si| Z[Fin] 
    C -->|No| D[Ir hacia la meta siguiendo la linea]
    D --> E{Obstaculo?}
    E -->|No| C
    E -->|Si| F[Recordar ubicacion del encuentro-hitpoint]
    F -->G[Seguir pared]
    G -->H{Linea-m o leave-point?}
    H -->|No| G
    H -->|Si| I{DistLeavePoint < DistHitpoint hasta meta?}
    I -->|Si| J[Dejar pared]
    I -->|No| G
    J --> D
```

# Reto Semana 5

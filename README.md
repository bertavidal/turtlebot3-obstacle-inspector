# turtlebot3-obstacle-inspector

### Project Description
This project implements autonomous obstacle-aware exploration and inspection for a TurtleBot3 robot using ROS.  
The robot moves forward while monitoring its frontal area with LiDAR. When an object is detected, it pauses and classifies the detection as either a wall-like structure or a discrete obstacle based on local scan density around the detected angle.

The system combines LiDAR measurements (distance and angle in robot coordinates) with odometry (x, y, yaw in world coordinates) to estimate obstacle positions in the global frame.  
It stores previously inspected obstacle positions and uses distance-based comparison to decide whether a newly detected obstacle is already known or truly new.

Behavior is organized as a finite-state process:
- Exploration: move forward until a detection occurs.
- Classification: determine wall vs obstacle.
- Avoidance: rotate to bypass walls and known obstacles.
- Inspection: for new obstacles, perform an in-place rotational scan and then resume exploration.

### Repository Structure
- `CMakeLists.txt`: Catkin/ROS build configuration and package target definition.
- `projecte_final_b.cpp`: Main C++ ROS node containing callbacks, state machine, navigation logic, obstacle memory, and inspection routine.
- `projecte_final_b_launch_file.launch`: ROS launch file to start the obstacle inspector node.
- `Informe Pràctica Final.pdf`: Final project report document.

---

### Descripció del projecte
Aquest projecte implementa exploració i inspecció autònoma amb consciència d’obstacles per a un robot TurtleBot3 utilitzant ROS.  
El robot avança mentre supervisa la zona frontal amb LiDAR. Quan detecta un objecte, s’atura i el classifica com a paret o com a obstacle discret segons la densitat local de mesures al voltant de l’angle detectat.

El sistema combina mesures del LiDAR (distància i angle en coordenades del robot) amb odometria (x, y, yaw en coordenades globals) per estimar la posició global dels obstacles.  
També desa les posicions dels obstacles ja inspeccionats i utilitza una comparació per distància per decidir si una detecció és coneguda o nova.

El comportament està organitzat com un procés de màquina d’estats:
- Exploració: avançar fins que hi hagi una detecció.
- Classificació: decidir paret vs obstacle.
- Evasió: girar per evitar parets i obstacles coneguts.
- Inspecció: per a obstacles nous, fer un escaneig rotacional in situ i reprendre l’exploració.

### Estructura del repositori 
- `CMakeLists.txt`: Configuració de compilació Catkin/ROS i definició del target del paquet.
- `projecte_final_b.cpp`: Node principal ROS en C++ amb callbacks, màquina d’estats, lògica de navegació, memòria d’obstacles i rutina d’inspecció.
- `projecte_final_b_launch_file.launch`: Fitxer de llançament ROS per iniciar el node inspector.
- `Informe Pràctica Final.pdf`: Document de l’informe final del projecte.

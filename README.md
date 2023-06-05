# Kallman_filter_Puzzlebot
Aplicacion del filtro de Kallman en el robot Puzzlebot de MR, por medio de una cámara y un rplidar

- Para poder correr el filtro debemos correr los siguientes comandos:
```
roslaunch puzzlebot_world puzzlebot_obstacle_world.launch
```
- Iniciamos el launch detector de los arucos:
```
roslaunch aruco_detect aruco_detect.launch fiducial_len:=0.33 image:=image_raw
```
- Iniciamos el codigo que calcula y mueve el robot con Kallman y evade obstaculos:
```
roslaunch solution Kallman_points.launch
```
- Todo esto lo tomamos en cuenta que ya cuentas con la librería de aruco detector.


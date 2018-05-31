Repositorio para guardar un trabajo simple para una asignatura de master.

Se requiere la prueba de varios algoritmos de planificación de trayectorias para compararlos entre ellos. Las herramientas en las que se basa el código son Gazebo, ROS y el robot Turtlebot. Los mapas en los que se prueban los algoritmos tienen un archivo .csv con los datos de los mismos. Estos documentos csv son cruciales para hacer la planificación.

Para comenzar el programa:
  1) Abrir mapa correspondiente en Gazebo
  2) Lanzar los nodos para el movimiento del robot
	roslaunch planificacion_pkg secuenciador.launch
  3) Lanzar el planificador escogido (solo uno) y marcando el mapa en el que se está probando el programa. Esto se hace porque en cada lanzador se elige el mapa csv que necesita abrir y los puntos x e y iniciales.
	roslaunch planificacion_pkg aestrella_X.launch
	roslaunch planificacion_pkg dij_X.launch
	roslaunch planificacion_pkg graf_dij_1.launch
		X es el número de mapa
  4) Publicar la casilla con el objetivo final en las coordenadas del mapa csv
	rostopic pub /goal_point

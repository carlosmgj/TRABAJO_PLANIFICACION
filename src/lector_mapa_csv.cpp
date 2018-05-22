#include <ros/ros.h>
#include "planificacion_pkg/lectura_mapa.h"
#include <fstream>

using namespace std;

bool salir = false;

bool servicio(planificacion_pkg::lectura_mapa::Request &req, planificacion_pkg::lectura_mapa::Response &res){
  
  geometry_msgs::Point punto;
  if (ros::param::get("/inicio_x", punto.x) && ros::param::get("/inicio_y", punto.y)) res.inicio=punto;
  else { res.inicio.x = 2; res.inicio.y = 2; }

  string line = "";
  int filas_contadas = 0;
  int columnas_contadas = 0;
  string file_name = "/home/josemanuel/master/robotica_movil/catkin_ws/src/planificacion/worlds/"+req.map_name+".csv";

  ifstream fichero(file_name.c_str());
	if (fichero.good()) ROS_INFO("Biieeeeeen");

  while (fichero.good() && getline(fichero, line))
  {
	stringstream strstr(line);
	string word = "";
	int resultado = -1;
	while (getline(strstr, word, char(0)))
	{
		filas_contadas++;
		//cout << word << endl;
		for (int j=0; j<word.size(); j=j+2)
		{
			if(filas_contadas==1) columnas_contadas++;

			if(word[j]=='1') resultado=1;
			else if (word[j]=='0') resultado=0;
			res.mapa.data.push_back(resultado);	
		}
	}
  }
  fichero.close();

  std_msgs::MultiArrayDimension dimension;

  dimension.label="filas";
  dimension.size = filas_contadas;
  dimension.stride=filas_contadas*columnas_contadas;
  res.mapa.layout.dim.push_back(dimension);

  dimension.label="columnas";
  dimension.size = columnas_contadas;
  dimension.stride= columnas_contadas;
  res.mapa.layout.dim.push_back(dimension);

  //res.mapa.layout.data_offset = 0;

  for (int f=0;f<filas_contadas*columnas_contadas;f++){
	if (f%columnas_contadas==0) cout << endl;
	cout << res.mapa.data[f];
  }
  cout << endl << "f: " << filas_contadas << endl;
  cout << "c: " << columnas_contadas << endl;
  salir = true;
  return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "lector_mapa_csv");
  ros::NodeHandle nodo;
  ros::ServiceServer service = nodo.advertiseService("leer_mapa",servicio);
  ros::Duration seconds_sleep(1);



  while (ros::ok && salir == false)
  {
	ros::spinOnce();
	seconds_sleep.sleep();
  }
}

#include <ros/ros.h>
#include <ros/ros.h>
#include "planificacion_pkg/lectura_mapa.h"
#include "planificacion_pkg/path.h"
#include <vector>

using namespace std;

class aestrella
{
public:
  aestrella()
  {
    //Topic you want to publish
    pub_ = nodo_.advertise<planificacion_pkg::path>("camino_offset",0);

    //Topic you want to subscribe
    sub_ = nodo_.subscribe("goal_point",0, &aestrella::crear_plan, this);
    //servicios
    serv_ = nodo_.serviceClient<planificacion_pkg::lectura_mapa>("leer_mapa");
  }

void lectura_mapa()
{
  planificacion_pkg::lectura_mapa srv_leer;
  
  string nombre_mapa;
  if (ros::param::get("/map_name", nombre_mapa)) srv_leer.request.map_name=nombre_mapa;
  else srv_leer.request.map_name = "map1";

  if (serv_.call(srv_leer))
  {
	mapa = srv_leer.response.mapa;
	ROS_INFO("Nodo creado. A la espera de datos");
	size = mapa.data.size();
	columnas = mapa.layout.dim[1].size;

	//A la espera de saber como vamos a enviar el dato de punto inicial, marco el punto inicial del map1

  	pos_inicial = srv_leer.response.inicio.x*columnas + srv_leer.response.inicio.y;
  }
  else ROS_INFO("Error al llamar al servicio");
}

void crear_plan(const geometry_msgs::Point::ConstPtr& msg){

  //Marcamos el punto final
  pos_final = msg->x*columnas + msg->y;

  if (pos_final>=size && msg->y < columnas) { if (mapa.data[pos_final] == 1) ROS_INFO("El punto no es valido");}
  else
  {

  //Se inicializan todos los vectores creados
  inicializar_datos();

  //Calculamos los costes de las casillas
  calcular_costes();
  //dibujar_costes();

 //Calculamos los heuristicos de las casillas
  calcular_heuristica();
  //dibujar_heuristica();
;
  // Calculamos el total
  calcular_total();
  dibujar_total();

  calcular_camino();

  publicar();
  }

}

void calcular_costes()
{

  bool completado = false;
  int pos_data = pos_final;
  while (!completado)  
  {
	checked[pos_data] = true;

	// comprobamos los cuadrantes alrededor del punto (pos_data). 
	for (int n = -1; n < 2; n++)  
	{
		for (int m = -1; m < 2; m++)
		{
			float coste_unitario; // Coste para moverse desde una casilla a otra
			if (n*m==0) coste_unitario = 1;
			else coste_unitario = 1.4;

			int pos_calcular = pos_data + columnas*n + m;
			if (pos_data != pos_calcular)
			{
				if ((mapa.data[pos_calcular] != 1) && checked[pos_calcular]==false )
				{
					if ( (costes[pos_calcular] == 0) || (costes[pos_calcular]>(costes[pos_data]+coste_unitario)) )
					{
						// Comprobar que no se choque con las esquinas
						if ((mapa.data[pos_calcular-m]==1) || (mapa.data[pos_calcular-n*columnas]==1)) continue;
						else costes[pos_calcular] = costes[pos_data] + coste_unitario;
					}
				}
				
			}
		}
	}
	//Calculamos la nueva posicion de pos_data
	bool activar_min = false;
	float min = 0;
	for (int i = 0; i < size; i++) 
	{

		if((costes[i] != 0) && (checked[i] == false))
		{
			if (activar_min == false)
			{
				min = costes[i];
				activar_min = true;
				pos_data = i;
			}
			else if (min > costes[i]) 
			{
				min = costes[i];
				pos_data = i;
			}
		}
			
	}
	// El bucle finaliza cuando se calculen todos los puntos del mapa
	if (min == 0) completado = true; 

  }
}

void calcular_heuristica()
{
  geometry_msgs::Point punto, end;
  end = data_to_point(pos_final, columnas);
  for (int i=0; i<size; i++)
  {
	if (mapa.data[i] != 1)
	{
		punto = data_to_point(i, columnas);
		heuristica[i] = sqrt(pow(punto.x -end.x,2) + pow(punto.y - end.y,2));
	}
  }
}

void calcular_total(){  for (int i=0; i<size; i++) total[i] = costes[i] + heuristica[i];}

geometry_msgs::Point data_to_point(int data, int c){

  geometry_msgs::Point punto;
  punto.y = data%c;
  punto.x = (data - punto.y) / c;
  punto.z = 0;

  return punto;
}

void inicializar_datos()
{
  checked.resize(size);
  heuristica.resize(size);
  costes.resize(size);
  total.resize(size);
  for (int i = 0; i<size; i++)
  {		
	if (mapa.data[i] == 1) checked[i] = 1;
	else checked[i] = false;	
	total[i] = 0;
	costes[i] = 0;
	heuristica[i] = 0;
  }
  camino.punto.clear();
  camino.punto.resize(0);
}

void dibujar_costes()
{
  for (int f=0;f<size;f++){
	if (f%columnas==0) cout << endl;
	cout << costes[f] << " -- ";
  }
  cout << endl;
}

void dibujar_heuristica()
{
  for (int f=0;f<size;f++){
	if (f%columnas==0) cout << endl;
	cout << heuristica[f] << " -- ";
  }
  cout << endl;
}

void dibujar_total()
{
  for (int f=0;f<size;f++){
	if (f%columnas==0) cout << endl;
	cout << total[f] << " -- ";
  }
  cout << endl;
}

void calcular_camino()
{
  vector<int>path;
  int pos_data = pos_inicial;
  int pos_calcular;
  bool activar_min;
  float min;
  int h;
  int i = 0;

  while (pos_data != pos_final)
  {
	activar_min = true;
	for (int n = -1; n < 2; n++)  
	{
		for (int m = -1; m < 2; m++)
		{
			pos_calcular = pos_data + columnas*n + m;
			if (pos_calcular == pos_data) continue;
			if (mapa.data[pos_calcular] == 1) continue;
			if (mapa.data[pos_calcular-m] == 1) continue;
			if (mapa.data[pos_calcular - n*columnas] == 1) continue;
			if (activar_min)
			{
				min = total[pos_calcular];
				h = heuristica[pos_calcular];
				path.push_back(pos_calcular);
				activar_min = false;
			}
			else if (min > total[pos_calcular])
			{
				min = total[pos_calcular];
				h = heuristica[pos_calcular];
				path[i] = pos_calcular;
			}
			else if ((min == total[pos_calcular]) && (h > heuristica[pos_calcular]))
			{
				min = total[pos_calcular];
				h = heuristica[pos_calcular];
				path[i] = pos_calcular;
			}
			
		}
	}
	pos_data = path[i];
	i++;
  }

  geometry_msgs::Point punto;
  for (int j = 0; j<i; j++)
  {
	punto = data_to_point(path[j],columnas);
	camino.punto.push_back(punto);
	cout << punto.x << " - " << punto.y << endl;
  }
}

void publicar() { 
	pub_.publish(camino); 
	pos_inicial = camino.punto[camino.punto.size()-1].x*columnas + camino.punto[camino.punto.size()-1].y;
}

private:
  ros::NodeHandle nodo_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber sub2_;
  ros::ServiceClient serv_;
  std_msgs::UInt16MultiArray mapa;

  vector<bool> checked;
  vector<float> costes;
  vector<float> heuristica;
  vector<float> total;


  int size;
  int columnas;
  int pos_inicial;
  int pos_final;
  planificacion_pkg::path camino;

};//End of class

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "a_estrella");

  //Create an object of class SubscribeAndPublish that will take care of everything
  aestrella planificador;

  ROS_INFO("Creando nodo planificador");

  planificador.lectura_mapa();
  ros::spin();

  ros::spin();

  return 0;
}

#include <ros/ros.h>
#include "planificacion_pkg/lectura_mapa.h"
#include "planificacion_pkg/path.h"
#include <vector>

using namespace std;

class dijkstra
{
public:
  dijkstra()
  {
    //Topic you want to publish
    pub_ = nodo_.advertise<planificacion_pkg::path>("camino_offset",0);
    pubp_ = nodo_.advertise<std_msgs::UInt16MultiArray>("data_mapa",0);
    //Topic you want to subscribe
    sub_ = nodo_.subscribe("goal_point",0, &dijkstra::crear_plan, this);
    //serv
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
	//int a; cin >> a;
	pubp_.publish(mapa);
	ROS_INFO("Publicado");
  }
  else ROS_INFO("Error al llamar al servicio");
}

void crear_plan(const geometry_msgs::Point::ConstPtr& msg){

  //Marcamos el punto final
  pos_final = msg->x*columnas + msg->y;

  if (pos_final>=size && msg->y < columnas) { if (mapa.data[pos_final] == 1) ROS_INFO("El punto no es valido");}
  else
  {

  ROS_INFO("Calculando trayectoria para llegar al punto (%i, %i)", int(msg->x), int(msg->y));

  //Se inicializan todos los vectores creados
  inicializar_datos();

  //Calculamos los costes de las casillas
  calcular_costes();
  dibujar_costes();
  //Calcular el camino a seguir
  calcular_camino();
  publicar();

; 
  }
}

void inicializar_datos()
{
  camino.punto.clear();
  camino.punto.resize(0);
  path.resize(size);
  costes.resize(size);
  checked.resize(size);
  for (int i = 0; i<size; i++)
  {			
	if (mapa.data[i] == 1) checked[i] = 1;
	else checked[i] = false;
	costes[i] = 0;
	path[i] = 0;
  }
}

void calcular_costes ()
{

  bool completado = false;
  int pos_calcular;
  int pos_data = pos_inicial;

  if (pos_data == pos_final) completado = true;
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

			pos_calcular = pos_data + columnas*n + m;
			if (pos_data != pos_calcular)
			{
				if ((mapa.data[pos_calcular] != 1) && checked[pos_calcular]==false )
				{
					if ( (costes[pos_calcular] == 0) || (costes[pos_calcular]>(costes[pos_data]+coste_unitario)) )
					{
						// Comprobar que no se choque con las esquinas
						if ((mapa.data[pos_calcular-m]==1) || (mapa.data[pos_calcular-n*columnas]==1)) continue;
						else costes[pos_calcular] = costes[pos_data] + coste_unitario;
						path[pos_calcular] = pos_data;
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
	
	if (pos_data == pos_final) completado = true; // El bucle finaliza cuando ya no se pueden calcular más caminos para llegar al destino
	if (min == 0) ROS_INFO ("ERROR: No se encuentra un camino posible");

  }
}

geometry_msgs::Point data_to_point(int data, int c){

  geometry_msgs::Point punto;
  punto.y = data%c;
  punto.x = (data - punto.y) / c;
  punto.z = 0;

  return punto;
}

void calcular_camino()
{
  int pos_calcular;
  pos_calcular = pos_final;
  geometry_msgs::Point punto;

  while (pos_calcular != pos_inicial)
  {
	punto = data_to_point(pos_calcular, columnas);
	pos_calcular = path[pos_calcular];
	camino.punto.push_back(punto);
  }

  //Como el camino está a la inversa (del ultimo al primero), se le da la vuelta.
  planificacion_pkg::path aux = camino;
  for (int f = 0; f<camino.punto.size(); f++) camino.punto[f] = aux.punto[camino.punto.size()-1-f];
  //for (int f = 0; f<camino.punto.size(); f++) cout << camino.punto[f].x << " - " << camino.punto[f].y << endl;
}

void dibujar_costes()
{
  for (int f=0;f<size;f++){
	if (f%columnas==0) cout << endl;
	cout << costes[f] << " -- ";
  }
  cout << endl;
}

void publicar() { 
	pub_.publish(camino); 
	pos_inicial = camino.punto[camino.punto.size()-1].x*columnas + camino.punto[camino.punto.size()-1].y;
}

private:
  ros::NodeHandle nodo_;
  ros::Publisher pub_;
  ros::Publisher pubp_;
  ros::Subscriber sub_;
  ros::ServiceClient serv_;
  std_msgs::UInt16MultiArray mapa;

  vector<int> path;
  vector<float> costes;
  vector<bool> checked;
  int size;
  int columnas;
  int pos_inicial;
  int pos_final;
  planificacion_pkg::path camino;


};//End of class


int main (int argc, char **argv)
{

  ros::init(argc, argv, "dijkstra");

  ROS_INFO("Creando nodo planificador");
  dijkstra planificador;

  planificador.lectura_mapa();
  ros::spin();



  return 0;
}

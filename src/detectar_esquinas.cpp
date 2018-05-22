#include <ros/ros.h>
#include "planificacion_pkg/modificar_mapa.h"


using namespace std;

bool servicio(planificacion_pkg::modificar_mapa::Request &req, planificacion_pkg::modificar_mapa::Response &res){

  res.resultante = req.inicial;
  int columnas = req.inicial.layout.dim[1].size;

  for (int d = 0; d<req.inicial.data.size(); d++)
  {
	if (req.inicial.data[d]==0)
	{
		//cout << req.inicial.data[d-columnas-1] << " " << req.inicial.data[d-1] << " " << req.inicial.data[d-columnas] << endl;
		if (req.inicial.data[d-columnas-1]==1) if(req.inicial.data[d-1] != 1 && req.inicial.data[d-columnas] != 1) { res.resultante.data[d] = 2; continue; }
		if (req.inicial.data[d-columnas+1]==1) if(req.inicial.data[d+1] != 1 && req.inicial.data[d-columnas] != 1) { res.resultante.data[d] = 2; continue; }
		if (req.inicial.data[d+columnas-1]==1) if(req.inicial.data[d-1] != 1 && req.inicial.data[d+columnas] != 1) { res.resultante.data[d] = 2; continue; }
		if (req.inicial.data[d+columnas+1]==1) if(req.inicial.data[d+1] != 1 && req.inicial.data[d+columnas] != 1) res.resultante.data[d] = 2;
	}
  }

  for (int f=0;f<res.resultante.data.size();f++){
	if (f%columnas==0) cout << endl;
	cout << res.resultante.data[f];

  }

  return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "detector_esquinas_nodo");
  ros::NodeHandle nodo;
  ros::ServiceServer service = nodo.advertiseService("esquinas_mapa",servicio);

  ros::spin();

  return 0;
}

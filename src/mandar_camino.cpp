#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "planificacion_pkg/path.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"mandar_camino");
	ros::NodeHandle nodo;
	ros::Publisher publicadorCamino = nodo.advertise<planificacion_pkg::path>("camino",0);
	planificacion_pkg::path caminito;
	geometry_msgs::Point punto;
	char respuesta='S';
        int i=0;
        string aux;
	while(ros::ok())
	{
		cout<<"pulsa para introducir camino nuevo"<<endl;
                cin>>aux;
		do
		{
                        cout<<"introduce la coordenada x para el punto"<<i<<endl;
			cin>>punto.x;
                        cout<<"introduce la coordenada y para el punto"<<i<<endl;
			cin>>punto.y;
                        caminito.punto.push_back(punto);
			cout<<"¿Continuar?(S/N)"<<endl;
			cin>>respuesta;
                        i=i+1;
			
                }while((respuesta=='S')or(respuesta=='s'));
                i=0;
		publicadorCamino.publish(caminito);
                caminito.punto.clear();

}
}

#include "ros/ros.h"

#include "std_msgs/Empty.h"
#include "geometry_msgs/Point.h"
#include "planificacion_pkg/path.h"

planificacion_pkg::path caminoaseguir;

using namespace std;
int i=0;
bool mover=false;
bool cerca=false;

void nuevo_camino(const planificacion_pkg::path::ConstPtr& msg)
{

    caminoaseguir.punto=msg->punto;

    ROS_INFO("Recibido  nuevo camino");
    cout<<caminoaseguir<<endl;
    mover=true;

}

void cercano(const std_msgs::Empty::ConstPtr& msg2)
{
    cerca=true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "secuenciador");
    ros::NodeHandle nodo;
 
    ROS_INFO("Nodo creado");
    ros::Publisher publicadorMensajes = nodo.advertise<geometry_msgs::Point>("goal_pose",0);
    ROS_INFO("Publicador y subscriptor creado");
    ros::Subscriber subscriptor = nodo.subscribe("camino",0,nuevo_camino);
    ros::Subscriber escucharespuesta = nodo.subscribe("respuesta",0,cercano);


    while (ros::ok())
    {
        if(mover==true)
        {
            for(i=0;i<caminoaseguir.punto.size();i++)
            {
                cout<<"mandando valor "<<i<<endl;
                publicadorMensajes.publish(caminoaseguir.punto[i]);
                while(cerca==false)
                {
                    ros::spinOnce();
                }
                cerca=false;
            }
            mover=false;
        }

	ros::spinOnce();
    }
}


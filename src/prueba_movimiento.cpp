#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "std_msgs/Int32.h"
using namespace std;

geometry_msgs::Point posicion;
geometry_msgs::Point posicion_goal;
geometry_msgs::Point posicion_inicial;
double vel_lineal=0.5;
double vel_angular=0.5;
double angulo;
double angulo_goal=0;
int direccion;


void girar();

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    movimiento = nodo.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);


    //Topic you want to subscribe
    odometria = nodo.subscribe("odom",1,&SubscribeAndPublish::odomCallback,this);
    angulo_objetivo = nodo.subscribe("goal_angle",1,&SubscribeAndPublish::goal_angleCallback,this);
    coordenadas_objetivo=nodo.subscribe("goal_pose",1,&SubscribeAndPublish::goal_poseCallback,this);

  }
  void goal_angleCallback(const std_msgs::Int32::ConstPtr& msg1)
  {
      angulo_goal=msg1->data;
  }
  void goal_poseCallback(const geometry_msgs::Point::ConstPtr& msg2)
  {
      posicion_goal.x=msg2->x;
      posicion_goal.y=msg2->y;
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      posicion.x = msg->pose.pose.position.x;
      posicion.y = msg->pose.pose.position.y;

      tf::Quaternion q(msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z,
                       msg->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      angulo = (360*yaw)/(2*3.14159);
      if (angulo < 0)
                angulo = angulo + 360;

  }

  ros::NodeHandle nodo;
  ros::Publisher movimiento;
  ros::Subscriber odometria;
  ros::Subscriber angulo_objetivo;
  ros::Subscriber coordenadas_objetivo;


};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"prueba_movimiento");    //Registrar el nombre del nodo
    geometry_msgs::Twist msgAEnviar;
    SubscribeAndPublish SAPObject;
    posicion_goal.x= posicion.x;
    posicion_goal.y=posicion.y;

    int intervalo=1;
    double hipotenusa;
    while(ros::ok)
    {


            if(angulo<angulo_goal-intervalo)                       //ANGULO ACTUAL MENOR QUE FINAL
            {
                  msgAEnviar.angular.z = 0.5;
                  if(angulo_goal-angulo>180)
                      msgAEnviar.angular.z=-msgAEnviar.angular.z;
            }
            else if(angulo>angulo_goal+intervalo)               //ANGULO ACTUAL MAYOR QUE FINAL
            {
                  msgAEnviar.angular.z = -0.5;
                  if(angulo-angulo_goal>180)
                     msgAEnviar.angular.z = -msgAEnviar.angular.z;
            }

            else
            {
                msgAEnviar.angular.z =0;
                hipotenusa=sqrt(pow((posicion_goal.x-posicion.x),2)+pow((posicion_goal.y-posicion.y),2));
                if(hipotenusa>0.05)
                {
                      msgAEnviar.linear.x = 0.5;
                      cout<<"No estoy en mi sitio"<<endl;
                }
                else
                {
                        msgAEnviar.linear.x = 0;
                        cout<<"estoy en mi sitio"<<endl;
                }
                cout<< "desde  ("<<posicion.x<<","<<posicion.y<<") hasta ("<<posicion_goal.x<<","<<posicion_goal.y<<") hay una distancia de:  "<<hipotenusa<<endl;
            }
            SAPObject.movimiento.publish(msgAEnviar);


        ros::spinOnce();
    }
}
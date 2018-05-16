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
    posicion_goal.x=0;
    posicion_goal.y=0;

    int intervalo=1;

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
                /*
                * Si el turtlebot esta por debajo del punto x e y ( abajo o izquierda) corremos hasta que llega un poco por debajo del punto
                  Si el turtlebot está por encima del punto x e y (encima o derecha) corremos hasya que llega un poco por endima del punto

                */
                if((posicion.x<posicion_goal.x)or(posicion.y<posicion_goal.y))
                {
                  if((posicion.x<posicion_goal.x-0.5)or(posicion.y<posicion_goal.y-0.5))
                  {
                    msgAEnviar.linear.x = 0.5;
                  }else
                  {
                  msgAEnviar.linear.x =0;
                  }
                }
                else if((posicion.x>posicion_goal.x)or(posicion.y>posicion_goal.y))
                {
                  if((posicion.x>posicion_goal.x+0.5)or(posicion.y>posicion_goal.y+0.5))
                  {
                    msgAEnviar.linear.x = 0.5;
                  }else
                  {
                  msgAEnviar.linear.x =0;
                  }
                }
            }
            SAPObject.movimiento.publish(msgAEnviar);


        ros::spinOnce();
    }
}

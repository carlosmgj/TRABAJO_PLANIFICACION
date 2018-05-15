#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

double angulo;
using namespace std;
void funcionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
tf::Quaternion q(msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z,
                       msg->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);

      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      angulo = (360*yaw)/(2*3.14159);
      if(angulo<0)
          angulo=angulo+360;
      cout<<angulo<<"               "<<msg->pose.pose.position.x<<"              "<<msg->pose.pose.position.y<<endl;
}

int main(int argc, char **argv){

    ros::init(argc,argv,"MostrarOdom");
    ros::NodeHandle nodo;
    ros::Subscriber subscriptor = nodo.subscribe("odom",0, funcionCallback);

    ros::spin();
    return 0;

}

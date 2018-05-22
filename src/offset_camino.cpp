#include <ros/ros.h>
#include "planificacion_pkg/path.h"

class offset
{
public:
  offset()
  {
    //Topic you want to publish
    pub_ = n_.advertise<planificacion_pkg::path>("camino", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("camino_offset", 1, &offset::callback, this);
  }

  void callback(const planificacion_pkg::path::ConstPtr& msg)
  {

    int off_x, off_y;

    geometry_msgs::Point punto;
    if (ros::param::get("/inicio_x", punto.x) && ros::param::get("/inicio_y", punto.y)) { off_x =punto.x; off_y = punto.y;}
    else { off_x = 2; off_y = 2; }

    planificacion_pkg::path output;
    for (int f = 0; f<msg->punto.size();f++){
	punto.x = msg->punto[f].x - off_x;
	punto.y = msg->punto[f].y - off_y;
	output.punto.push_back(punto);
    }
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "offset_camino");

  //Create an object of class SubscribeAndPublish that will take care of everything
  offset camino;

  ros::spin();

  return 0;
}

#include "ros/ros.h"
#include "ARM5PowerControl/onOff.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "powerClient");
  if (argc != 2)
  {
    ROS_INFO("usage: powerClient onOff ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ARM5PowerControl::onOff>("powerServer");
  ARM5PowerControl::onOff srv;
  srv.request.onOff = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("%d",srv.response.confirm);
    if(srv.response.confirm==1 && srv.response.confirm==srv.request.onOff){
      ROS_INFO("Operacion correcta");
      ROS_INFO("Fuente de alimentacion encendida");
    }
    else if(srv.response.confirm==2 && srv.response.confirm==srv.request.onOff){
      ROS_INFO("Operacion correcta");
      ROS_INFO("Fuente de alimentacion apagada");
    }
      
    else
      ROS_INFO("Operacion incorrecta");
  }
  else
  {
    ROS_ERROR("Failed to call service IO");
    return 1;
  }

  return 0;
}

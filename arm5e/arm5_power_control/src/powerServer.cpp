#include "ros/ros.h"
#include "arm5_power_control/onOff.h"
#include "stdio.h"
#include "string.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int fd;

bool onOffFunction(arm5_power_control::onOff::Request &req, arm5_power_control::onOff::Response &res){
  int iout;
  char b[1];
  iout=write(fd, &req.onOff, 1);
  if(iout!=1){
    ROS_INFO("Error al escribir.");
  return false;
  }
  sleep(1);
  iout=-1;
 // while(iout==-1){
    iout=read(fd, b, 1);
 // }
  res.confirm=1;
  return true;
}


int main(int arg, char **argv){
 
  
  fd=open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  if(fd==-1){
    ROS_INFO("Puerto cerrado.");
    return -1;
  }
  struct termios my_termios;
   if((tcgetattr(fd, &my_termios))>=0){
      speed_t brate=B9600;
      cfsetispeed(&my_termios, brate);
      cfsetospeed(&my_termios, brate);
      
      my_termios.c_cflag &= ~PARENB;
      my_termios.c_cflag &= ~CSTOPB;
      my_termios.c_cflag &= ~CSIZE;
      my_termios.c_cflag |= CS8;
      my_termios.c_cflag &= ~CRTSCTS;
      my_termios.c_cflag |= CREAD | CLOCAL;
      my_termios.c_iflag &= ~(IXON | IXOFF | IXANY);
      my_termios.c_oflag &= ~OPOST;
      my_termios.c_cc[VMIN]=1;
      my_termios.c_cc[VTIME]=0;
      if( tcsetattr(fd, TCSANOW, &my_termios)<0){
	ROS_INFO("Error al cambiar los atributos del puerto.");
	return -1;
      }
   }
   else{
     ROS_INFO("Error al obtener los atributos del puerto.");
     return -1;
   }
   ros::init(arg, argv, "powerServer");
   ros::NodeHandle n;
   ros::ServiceServer service = n.advertiseService("onOff", onOffFunction);
   ros::spin();
   return 0;
}

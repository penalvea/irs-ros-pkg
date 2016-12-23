#include "ros/ros.h"
#include "optoforce/optoforce6DSensor.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "optoforce6DSensor");
	int iPortIndex = 0;  // The index of the port which will be opened
	int iSpeed = 333; // Speed in Hz
	int iFilter = 15;  // Filter in Hz
	ros::NodeHandle nh;
	Optoforce6DSensor sensor(nh, iPortIndex, iSpeed, iFilter);
	sensor.run();
	return 0;
}

#ifndef OPTOFORCE6DSENSOR_H
#define OPTOFORCE6DSENSOR_H

#include "ros/ros.h"
#include "omd/opto.h"
#include "omd/sensorconfig.h"
#include "omd/optopackage.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_srvs/Empty.h"

class Optoforce6DSensor{
	ros::NodeHandle nh_;
	OptoDAQ optoDaq_;
	OptoPorts optoPorts_;
	OptoPackage6D lastPkg_;
	int iPortIndex_;
	int iSpeed_;
	int iFilter_;
	ros::Publisher optoPub_;
	ros::ServiceServer setZeroSrv_;

	bool setSensorZero(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool setConfig();
	bool openPort();
	int readPackage6D();

public:
	Optoforce6DSensor(ros::NodeHandle &nh, int iPortIndex, int iSpeed, int iFilter);
	~Optoforce6DSensor(){optoDaq_.close();};
	void run();


};

#endif

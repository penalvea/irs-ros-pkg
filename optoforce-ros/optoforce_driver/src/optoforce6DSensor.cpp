#include "optoforce_driver/optoforce6DSensor.h"


Optoforce6DSensor::Optoforce6DSensor(ros::NodeHandle& nh, int iPortIndex, int iSpeed, int iFilter){
	nh_=nh;
	iPortIndex_=iPortIndex;
	iSpeed_=iSpeed;
	iFilter_=iFilter;
	optoPub_=nh_.advertise<geometry_msgs::WrenchStamped>("optoforce6DSensor", 1);
	setZeroSrv_=nh_.advertiseService("optoforce6DSensor/setSensorZero", &Optoforce6DSensor::setSensorZero, this);
}


bool Optoforce6DSensor::setSensorZero(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	optoDaq_.zeroAll();
	return true;

}


bool Optoforce6DSensor::setConfig(){
	SensorConfig sensorConfig;
	sensorConfig.setSpeed(iSpeed_);
	sensorConfig.setFilter(iFilter_);

	int tries=0;
	while(tries<10){
		if(optoDaq_.sendConfig(sensorConfig)){
			return true;
		}
		usleep(1000);
		tries++;
	}
	return false;
}
bool Optoforce6DSensor::openPort(){
	usleep(2500000);

	OPort* portList=optoPorts_.listPorts(true);
	int iLastSize=optoPorts_.getLastSize();
	if(iPortIndex_>iLastSize){
		return false;
	}
	if(optoDaq_.open(portList[iPortIndex_])){
		std::string deviceName = std::string(portList[iPortIndex_].deviceName);
		std::string name = std::string(portList[iPortIndex_].name);
		std::string serialNumber = std::string (portList[iPortIndex_].serialNumber);
		int version = optoDaq_.getVersion();
		std::cout<<"Device Name: "<<deviceName<<std::endl;
		std::cout<<"Name: "<<name<<std::endl;
		std::cout<<"Serial Number: "<<serialNumber<<std::endl;
		std::cout<<"Version: "<<version<<std::endl;
		return true;
	}
	return false;

}
int Optoforce6DSensor::readPackage6D(){
	return optoDaq_.read6D(lastPkg_, false);

}
void Optoforce6DSensor::run(){
	if(!openPort()){
		std::cout<<"Could not open port"<<std::endl;
		return;
	}
	if(!setConfig()){
		std::cout<<"Could not set the configuration"<<std::endl;
		return;
	}
	opto_version optoVersion=optoDaq_.getVersion();
	if(optoVersion!=_95 && optoVersion!=_64){
		std::cout<<"Error: 3D sensor"<<std::endl;
		return;
	}
	while(ros::ok()){
		int iReadSize = readPackage6D();
		if (iReadSize < 0) {
			std::cout<<"Something went wrong, DAQ closed!"<<std::endl;
			return;
		}
		if(iReadSize>0){
			geometry_msgs::WrenchStamped msg;
			msg.header.stamp=ros::Time::now();
			msg.wrench.force.x=lastPkg_.Fx;
			msg.wrench.force.y=lastPkg_.Fy;
			msg.wrench.force.z=lastPkg_.Fz;
			msg.wrench.torque.x=lastPkg_.Tx;
			msg.wrench.torque.y=lastPkg_.Ty;
			msg.wrench.torque.z=lastPkg_.Tz;
			optoPub_.publish(msg);
		}
		ros::spinOnce();

	}

}

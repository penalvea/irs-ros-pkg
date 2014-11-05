#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRotationMatrix.h>
#include <ros/ros.h>
#include <string.h>

namespace mar_params {

vpHomogeneousMatrix paramToVispHomogeneousMatrix(ros::NodeHandle *nh, std::string param) {
	XmlRpc::XmlRpcValue v;
	vpHomogeneousMatrix m;
	if (nh->getParam(param,v) && v.size()==6) {
		double e[6];
		for (int i=0; i<6; i++) {
			XmlRpc::XmlRpcValue value=v[i];
			e[i]=static_cast<double>(value);
		}
		m.buildFrom(vpTranslationVector(e[0],e[1],e[2]), vpRotationMatrix(vpRxyzVector(e[3],e[4],e[5])));
	}
	else if(nh->getParam(param,v) && v.size()==7){
		double e[7];
		for (int i=0; i<7; i++){
			XmlRpc::XmlRpcValue value=v[i];
			e[i]=static_cast<double>(value);
		}
		m.buildFrom(vpTranslationVector(e[0],e[1],e[2]), vpQuaternionVector(e[3],e[4],e[5],e[6]));
	}
	else {
		ROS_WARN("Parameter %s not found or not of correct dimension", param.c_str());
	}
	return m;
}

vpColVector paramToVispColVector(ros::NodeHandle *nh, std::string param) {
	XmlRpc::XmlRpcValue v;
	vpColVector vector;
	if (nh->getParam(param,v) && v.size()>0) {
		vector.resize(v.size());
		//vpColVector vector(v.size());
		for (int i=0; i<v.size(); i++) {
			XmlRpc::XmlRpcValue value=v[i];
			vector[i]=static_cast<double>(value);
		}
	} else {
		ROS_WARN("Parameter %s not found or not of correct dimension", param.c_str());
	}
	return vector;
}
}


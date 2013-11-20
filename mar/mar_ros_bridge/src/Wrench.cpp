/*
 * Wrench.cpp
 *
 *  Created on: 01/08/2012
 *      Author: mprats
 */

#include <mar_ros_bridge/Wrench.h>

vpColVector Wrench::getWrench() {
	vpColVector t(6);

	t[0]=msg_.force.x;
	t[1]=msg_.force.y;
	t[2]=msg_.force.z;
	t[3]=msg_.torque.x;
	t[4]=msg_.torque.y;
	t[5]=msg_.torque.z;

	return t;
}



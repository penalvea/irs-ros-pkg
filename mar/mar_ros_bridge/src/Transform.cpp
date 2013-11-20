/*
 * Transform.cpp
 *
 *  Created on: 01/08/2012
 *      Author: mprats
 */


#include <mar_ros_bridge/Transform.h>
#include <kdl/frames.hpp>

vpHomogeneousMatrix Transform::getTransform() {
	KDL::Rotation vRek;
	vRek=KDL::Rotation::Quaternion(msg_.rotation.x, msg_.rotation.y, msg_.rotation.z, msg_.rotation.w);

	vpRotationMatrix vRe;
	memcpy(vRe.data, vRek.data,9*sizeof(double));
	vpTranslationVector vTe(msg_.translation.x, msg_.translation.y, msg_.translation.z);

	return vpHomogeneousMatrix(vTe, vRe);
}



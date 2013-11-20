/*
 * robot_utils.cpp
 *
 *  Created on: 01/08/2012
 *      Author: mprats
 */


#include <mar_robot/robot_utils.h>
#include <kdl/frames.hpp>

#include <visp/vpRotationMatrix.h>
#include <visp/vpTranslationVector.h>

namespace mar_robot_utils {

vpHomogeneousMatrix transformToVisp(const geometry_msgs::Transform &t) {
	KDL::Rotation vRek;
	vRek=KDL::Rotation::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);

	vpRotationMatrix vRe;
	memcpy(vRe.data, vRek.data,9*sizeof(double));
	vpTranslationVector vTe(t.translation.x, t.translation.y, t.translation.z);

	return vpHomogeneousMatrix(vTe, vRe);
}

}

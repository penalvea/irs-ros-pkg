/*
 * robot_utils.h
 *
 *  Created on: 01/08/2012
 *      Author: mprats
 */

#ifndef ROBOT_UTILS_H_
#define ROBOT_UTILS_H_

#include <geometry_msgs/Transform.h>
#include <visp/vpHomogeneousMatrix.h>

namespace mar_robot_utils {

vpHomogeneousMatrix transformToVisp(const geometry_msgs::Transform &t);

}

#endif /* ROBOT_UTILS_H_ */

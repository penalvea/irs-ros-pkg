/*
 * Odometry.cpp
 *
 *  Created on: 21/06/2012
 *      Author: mprats
 */


#include <mar_ros_bridge/Odometry.h>
#include <tf/transform_datatypes.h>

vpHomogeneousMatrix Odometry::getPose() {
	//TODO: Eventually move to or use visp_bridge
    vpHomogeneousMatrix mat;
    vpTranslationVector vec(msg_.pose.pose.position.x,msg_.pose.pose.position.y,msg_.pose.pose.position.z);
    vpRotationMatrix rmat;

    double a = msg_.pose.pose.orientation.x;
    double b = msg_.pose.pose.orientation.y;
    double c = msg_.pose.pose.orientation.z;
    double d = msg_.pose.pose.orientation.w;

    rmat[0][0] = a*a+b*b-c*c-d*d;
    rmat[0][1] = 2*b*c-2*a*d;
    rmat[0][2] = 2*a*c+2*b*d;

    rmat[1][0] = 2*a*d+2*b*c;
    rmat[1][1] = a*a-b*b+c*c-d*d;
    rmat[1][2] = 2*c*d-2*a*b;

    rmat[2][0] = 2*b*d-2*a*c;
    rmat[2][1] = 2*a*b+2*c*d;
    rmat[2][2] = a*a-b*b-c*c+d*d;

    mat.buildFrom(vec,rmat);

    return mat;
}

vpColVector Odometry::getTwist() {
	vpColVector t(6);

	t[0]=msg_.twist.twist.linear.x;
	t[2]=msg_.twist.twist.linear.y;
	t[3]=msg_.twist.twist.linear.z;
	t[4]=msg_.twist.twist.angular.x;
	t[5]=msg_.twist.twist.angular.y;
	t[6]=msg_.twist.twist.angular.z;

	return t;
}

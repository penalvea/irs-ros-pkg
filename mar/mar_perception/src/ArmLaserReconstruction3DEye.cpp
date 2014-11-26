/*
 * ArmLaserReconstruction3DEye.cpp
 *
 *  Created on: 13/10/2014
 *      Author: toni
 */

#include "mar_perception/ArmLaserReconstruction3DEye.h"
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpPixelMeterConversion.h>



void ArmLaserReconstruction3DEye::perceive() {

	vpColVector q;
	arm_->getJointValues(q);
	q[3]=0;
	q[4]=0;
	vpHomogeneousMatrix bMe;
	ros::spinOnce();
	ros::Time begin=ros::Time::now();
	bMe=arm_->directKinematics(q);
	laser_detector_->setLimits(bMe*eMc, bMe*eMl);
	laser_detector_->perceive();
	ros::Duration duration=ros::Time::now()-begin;
	duration_+=duration;
	times_++;
	if(times_==150){
		std::cout<<"eye seconds="<<duration_.toSec()<<std::endl;
		std::cout<<"eye nseconds="<<duration_.toNSec()<<std::endl;
	}



	//arm_->getPosition(bMe);
	cMl=eMc.inverse()*eMl;
	vpRotationMatrix cRl;
	cMl.extract(cRl);
	vpColVector pn_l(3), pn_c(3), pp(3);    //plane normal (laser), plane normal (camera) and plane point

	pn_l[0]=1; pn_l[1]=0; pn_l[2]=0;
	pp[0]=cMl[0][3]; pp[1]=cMl[1][3]; pp[2]=cMl[2][3];
	pn_c=cRl*pn_l;


	for (unsigned int i=0; i<laser_detector_->points.size(); i++) {
		//Reconstruct 3D point
		//Intersect (u,v) ray with the laser plane
		vpColVector ld(3);
		vpPixelMeterConversion::convertPoint(laser_detector_->getGrabber()->K,laser_detector_->points[i][1],laser_detector_->points[i][0],ld[0],ld[1]);
		ld[2]=1;
		double lambda=vpColVector::dotProd(pn_c,pp)/vpColVector::dotProd(pn_c,ld);
		vpColVector point3d(4);
		vpHomogeneousMatrix cMp(lambda*ld[0], lambda*ld[1], lambda*ld[2],0,0,0);

		vpHomogeneousMatrix bMp=bMe*eMc*cMp;
		point3d[0]=bMp[0][3];
		point3d[1]=bMp[1][3];
		point3d[2]=bMp[2][3];
		point3d[3]=1;





		points3d.push_back(point3d);


	}
}




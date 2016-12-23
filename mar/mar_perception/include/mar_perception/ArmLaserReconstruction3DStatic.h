/*
 * ArmLaserReconstruction3DStatic.h
 *
 *  Created on: 13/10/2014
 *      Author: toni
 */

#ifndef ARMLASERRECONSTRUCTION3DSTATIC_H_
#define ARMLASERRECONSTRUCTION3DSTATIC_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <mar_core/CPerception.h>
#include <mar_core/Arm.h>
#include <mar_perception/LaserPeakDetector.h>
#include <mar_perception/VirtualImage.h>
#include <mar_perception/Reconstruction3D.h>

#include <boost/shared_ptr.hpp>

/* 3D Reconstruction from an arm-mounted laser in a mobile platform
 * Estimates the motion of the platform and the arm holding the laser for relating all the points
 * with respect to a fixed frame
 * */

class ArmLaserReconstruction3DStatic: public Reconstruction3D {
	boost::shared_ptr<LaserPeakDetector> laser_detector_;
	//boost::shared_ptr<Arm> arm_;
	ARM5ArmPtr arm_;

	vpHomogeneousMatrix bMc, cMb;	///< Camera wrt arm base and vice versa
	vpHomogeneousMatrix eMl;                ///< Laser frame wrt end-effector frame
	ros::Duration duration_;
	int times_;



public:
/*	ArmLaserReconstruction3DStatic(LaserPeakDetectorPtr laser_detector, ARM5ArmPtr arm, ESMTrackingPtr esm, MotionEstimatorPtr mest): Reconstruction3D() {
		laser_detector_=laser_detector;
		arm_=arm;
		esm_=esm;
		mest_=mest;
	}*/
	ArmLaserReconstruction3DStatic(LaserPeakDetectorPtr laser_detector, ARM5ArmPtr arm): Reconstruction3D() {
		laser_detector_=laser_detector;
		arm_=arm;
		times_=0;

	}


	void perceive();

	/** Sets the relationship between the camera frame and the arm base frame */
	void setCameraToBase(vpHomogeneousMatrix bMc) {
		this->bMc=bMc;
		cMb=bMc.inverse();
	}

	/** Sets the relationship between the laser frame and the end-effector frame */
	void setLaserToEef(vpHomogeneousMatrix eMl) {
		this->eMl=eMl;
	}

	LaserPeakDetectorPtr getPeakDetector() {return laser_detector_;}
	int getIterations(){return times_;}
	ros::Duration getDuration(){return duration_;}

	virtual ~ArmLaserReconstruction3DStatic(){}
};

typedef boost::shared_ptr<ArmLaserReconstruction3DStatic> ArmLaserReconstruction3DStaticPtr;

#endif /* ARMLASERRECONSTRUCTION3DSTATIC_H_ */

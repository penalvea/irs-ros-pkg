/*
 * ArmLaserReconstruction3DEye.h
 *
 *  Created on: 13/10/2014
 *      Author: toni
 */



#ifndef ARMLASERRECONSTRUCTION3DEYE_H_
#define ARMLASERRECONSTRUCTION3DEYE_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <mar_core/CPerception.h>
#include <mar_core/Arm.h>
#include <mar_perception/LaserPeakDetector.h>
#include <mar_perception/ESMTracking.h>
#include <mar_perception/VirtualImage.h>
#include <mar_perception/MotionEstimator.h>
#include <mar_perception/Reconstruction3D.h>
#include <mar_robot_arm5e/ARM5Arm.h>

#include <boost/shared_ptr.hpp>

class ArmLaserReconstruction3DEye: public Reconstruction3D {

	boost::shared_ptr<LaserPeakDetector> laser_detector_;
	//boost::shared_ptr<Arm> arm_;
	ARM5ArmPtr arm_;
	boost::shared_ptr<ESMTracking> esm_;
	boost::shared_ptr<MotionEstimator> mest_;

	vpHomogeneousMatrix eMc;	    ///< camera frame wrt end-effector frame
	vpHomogeneousMatrix eMl;		///< Laser frame wrt end-effector frame
	vpHomogeneousMatrix cMl;		///< Laser frame wrt camerar frame
	ros::Duration duration_;
	int times_;

public:
	ArmLaserReconstruction3DEye(LaserPeakDetectorPtr laser_detector, ARM5ArmPtr arm): Reconstruction3D() {
		laser_detector_=laser_detector;
		arm_=arm;
		times_=0;
		//duration_=0;

	}
	void perceive();


	/** Sets the relationship between the laser frame and the end-effector frame */
	void setLaserToEef(vpHomogeneousMatrix eMl) {
		this->eMl=eMl;
	}

	/** Sets the relationship between the camera frame and the end-effector frame */
	void setCameraToEef(vpHomogeneousMatrix eMc){
		this->eMc=eMc;
	}

	LaserPeakDetectorPtr getPeakDetector() {return laser_detector_;}
	virtual ~ArmLaserReconstruction3DEye() {}
};

typedef boost::shared_ptr<ArmLaserReconstruction3DEye> ArmLaserReconstruction3DEyePtr;

#endif /* ARMLASERRECONSTRUCTION3DEYE_H_ */

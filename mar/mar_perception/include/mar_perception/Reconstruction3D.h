/*
 * Reconstruction3D.h
 *
 *  Created on: 24/05/2012
 *      Author: mprats
 */

#ifndef RECONSTRUCTION3D_H_
#define RECONSTRUCTION3D_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <mar_core/CPerception.h>
#include <mar_core/Arm.h>
#include <mar_perception/LaserPeakDetector.h>
#include <mar_perception/ESMTracking.h>
#include <mar_perception/VirtualImage.h>
#include <mar_perception/MotionEstimator.h>

#include <boost/shared_ptr.hpp>

/** Abstract class for 3D reconstruction algorithms */
class Reconstruction3D : public CPerception {
public:
	std::vector<vpColVector> points3d;	///< A vector with all the 3D points
	std::vector<vpColVector> points3d_color;	///< A vector with all the 3D points colors

protected:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;	///< The point cloud in PCL format

	void buildPCLCloudFromPoints();
public:
	Reconstruction3D(): CPerception() {
		cloud_.reset();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud() {
		if (!cloud_) buildPCLCloudFromPoints();
		return cloud_;}

	/** Sense and accumulate 3D points into the cloud */
	void perceive()=0;

	void reset() {points3d.clear();}

	void savePCD(std::string filename);

	virtual ~Reconstruction3D() {}
};
typedef boost::shared_ptr<Reconstruction3D> Reconstruction3DPtr;


/* 3D Reconstruction from an arm-mounted laser in a mobile platform
 * Estimates the motion of the platform and the arm holding the laser for relating all the points
 * with respect to a fixed frame
 * */
class ArmLaserReconstruction3D : public Reconstruction3D {
	boost::shared_ptr<LaserPeakDetector> laser_detector_;
	boost::shared_ptr<Arm> arm_;
	boost::shared_ptr<ESMTracking> esm_;
	boost::shared_ptr<VirtualImage> grabber_;
	boost::shared_ptr<MotionEstimator> mest_;

	vpHomogeneousMatrix bMc, cMb;	///< Camera wrt arm base and vice versa
	vpHomogeneousMatrix eMl;		///< Laser frame wrt end-effector frame

	vpImage<vpRGBa> Iref_;

public:
	ArmLaserReconstruction3D(LaserPeakDetectorPtr laser_detector, ArmPtr arm, ESMTrackingPtr esm,
			MotionEstimatorPtr mest, VirtualImagePtr grabber): Reconstruction3D() {
		laser_detector_=laser_detector;
		arm_=arm;
		esm_=esm;
		mest_=mest;
		grabber_=grabber;
		Iref_=grabber->image;
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

	virtual ~ArmLaserReconstruction3D() {}
};
typedef boost::shared_ptr<ArmLaserReconstruction3D> ArmLaserReconstruction3DPtr;


#endif /* RECONSTRUCTION3D_H_ */

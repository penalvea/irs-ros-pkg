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
#include <mar_perception/VirtualImage.h>


#include <boost/shared_ptr.hpp>


/** Abstract class for 3D reconstruction algorithms */
class Reconstruction3D : public CPerception {
public:
	std::vector<vpColVector> points3d;	///< A vector with all the 3D points

protected:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;	///< The point cloud in PCL format

	void buildPCLCloudFromPoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr buildPCLCloudFromPointsBetweenIndexes(int index_min, int index_max);
public:
	Reconstruction3D(): CPerception() {
		cloud_.reset();

	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() {
		if (!cloud_)
			buildPCLCloudFromPoints();
		return cloud_;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getLimitedCloud(int index_min, int index_max) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		cloud=buildPCLCloudFromPointsBetweenIndexes(index_min, index_max);
		return cloud;

	}

	/** Sense and accumulate 3D points into the cloud */
	void perceive()=0;
	virtual LaserPeakDetectorPtr getPeakDetector()=0;
	virtual int getIterations()=0;
	virtual ros::Duration getDuration()=0;

	void reset() {points3d.clear();
	cloud_.reset();}

	void savePCD(std::string filename);


	virtual ~Reconstruction3D() {
	}
};
typedef boost::shared_ptr<Reconstruction3D> Reconstruction3DPtr;


#endif /* RECONSTRUCTION3D_H_ */

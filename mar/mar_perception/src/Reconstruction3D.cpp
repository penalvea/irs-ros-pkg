/*
 * Reconstruction3D.cpp
 *
 *  Created on: 24/05/2012
 *      Author: mprats
 */

#include <mar_perception/Reconstruction3D.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpPixelMeterConversion.h>



void Reconstruction3D::buildPCLCloudFromPoints() {
	cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_->points.resize (points3d.size());
	cloud_->height = 1;
	cloud_->width = points3d.size();
	cloud_->sensor_origin_[0] = 0;
	cloud_->sensor_origin_[1] = 0;
	cloud_->sensor_origin_[2] = 0;
	cloud_->sensor_orientation_.w () = 0;
	cloud_->sensor_orientation_.x () = 0;
	cloud_->sensor_orientation_.y () = 0;
	cloud_->sensor_orientation_.z () = 0;
	for (unsigned int i=0; i<points3d.size(); i++) {
		cloud_->points[i].x = points3d[i][0];
		cloud_->points[i].y = points3d[i][1];
		cloud_->points[i].z = points3d[i][2];
	}
}

void Reconstruction3D::savePCD(std::string filename) {
	if (!cloud_) buildPCLCloudFromPoints();
	pcl::io::savePCDFile(filename, *cloud_, false);
}

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

void ArmLaserReconstruction3D::perceive() {
	//TODO from laserPeakDetector.cpp example
	vpHomogeneousMatrix bMe;
	arm_->getPosition(bMe);

	vpHomogeneousMatrix cMl=cMb*bMe*eMl;
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
		vpPixelMeterConversion::convertPoint(grabber_->K,laser_detector_->points[i][1],laser_detector_->points[i][0],ld[0],ld[1]);
		ld[2]=1;

		double lambda=vpColVector::dotProd(pn_c,pp)/vpColVector::dotProd(pn_c,ld);
		vpColVector point3d(4), point3d_color(3);
		point3d[0]=lambda*ld[0];
		point3d[1]=lambda*ld[1];
		point3d[2]=lambda*ld[2];
		point3d[3]=1;
		if (mest_)
			point3d=mest_->getMotionEstimation()*point3d;

		point3d_color[0]=Iref_(laser_detector_->points[i][0],laser_detector_->points[i][1]).R;
		point3d_color[1]=Iref_(laser_detector_->points[i][0],laser_detector_->points[i][1]).G;
		point3d_color[2]=Iref_(laser_detector_->points[i][0],laser_detector_->points[i][1]).B;

		points3d.push_back(point3d);
		points3d_color.push_back(point3d_color);
	}

}

void Reconstruction3D::buildPCLCloudFromPoints() {
	cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
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

		// Pack the RGB values in
		// increase red with increasing x, increase green with increasing y
		uint32_t rgb_val_;
		rgb_val_ = ((float) points3d_color[i][0]); // RED
		rgb_val_ = rgb_val_ << 8;
		rgb_val_ += ((float) points3d_color[i][1]); // GREEN
		rgb_val_ = rgb_val_ << 8;
		rgb_val_ += ((float) points3d_color[i][2]); // BLUE
		memcpy(&(cloud_->points[i].rgb), &rgb_val_, sizeof(float));
	}
}

void Reconstruction3D::savePCD(std::string filename) {
	if (!cloud_) buildPCLCloudFromPoints();
	pcl::io::savePCDFile(filename, *cloud_, false);
}

/*
 * PCGraspPlanning.cpp
 *
 *  Created on: 07/06/2012
 *      Author: mprats
 */

#include <ros/ros.h>

#include <mar_perception/PCGraspPlanning.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>

void PCGraspPlanning::perceive() {
	//Find mean Z. Used for knowing the real dimension (meters) of the clicked line
	double meanz=0;
	for (size_t i = 0; i < cloud_->points.size (); ++i)
		meanz+=cloud_->points[i].z;
	meanz/=cloud_->points.size();
	ROS_INFO_STREAM("mean Z is: " << meanz);

	//Intersect clicked points with the meanZ plane
	vpColVector ip1m(3), ip2m(3);
	vpPixelMeterConversion::convertPoint(k_,g1_.get_u(),g1_.get_v(),ip1m[0],ip1m[1]);
	ip1m[2]=1;
	vpPixelMeterConversion::convertPoint(k_,g2_.get_u(),g2_.get_v(),ip2m[0],ip2m[1]);
	ip2m[2]=1;
	ip1m*=meanz;
	ip2m*=meanz;

	ROS_INFO_STREAM("ip1m is: " << ip1m.t());
	ROS_INFO_STREAM("ip2m is: " << ip2m.t());

	double d1=(ip2m-ip1m).euclideanNorm();
	double d2=hand_width_;

	//Compute the bounding box corners in a local frame
	vpColVector p1(4),p2(4),p3(4),p4(4);
	p1[0]=-d2/2; p1[1]=-d1/2; p1[2]=0; p1[3]=1;
	p2[0]=d2/2; p2[1]=-d1/2; p2[2]=0; p2[3]=1;
	p3[0]=d2/2; p3[1]=d1/2; p3[2]=0; p3[3]=1;
	p4[0]=-d2/2; p4[1]=d1/2; p4[2]=0; p4[3]=1;

	//Transform from the local frame to the camera frame
	double alpha=atan2((ip2m-ip1m)[1],(ip2m-ip1m)[0])-M_PI_2;
	ROS_INFO_STREAM("Angle is: " << alpha);
	vpHomogeneousMatrix cMbb((ip1m[0]+ip2m[0])/2, (ip1m[1]+ip2m[1])/2, meanz,0,0,alpha);
	ROS_INFO_STREAM("cMbb is: " << std::endl << cMbb);

	p1=cMbb*p1;
	p2=cMbb*p2;
	p3=cMbb*p3;
	p4=cMbb*p4;
	std::cerr << "p1 is " << p1.t() << std::endl;
	std::cerr << "p2 is " << p2.t() << std::endl;
	std::cerr << "p3 is " << p3.t() << std::endl;
	std::cerr << "p4 is " << p4.t() << std::endl;
	
	vpPoint bbplot[4];
	bbplot[0].setWorldCoordinates(p1[0],p1[1],p1[2]);
	bbplot[1].setWorldCoordinates(p2[0],p2[1],p2[2]);
	bbplot[2].setWorldCoordinates(p3[0],p3[1],p3[2]);
	bbplot[3].setWorldCoordinates(p4[0],p4[1],p4[2]);

//	for (unsigned int i=0; i<4; i++) {
	//	bbplot[i].display(I_,vpHomogeneousMatrix(),k_,vpColor::blue);
	//}
	//vpDisplay::flush(I_);


	double minx, miny, maxx, maxy;
	minx=p1[0]; miny=p1[1]; maxx=p1[0]; maxy=p1[1];
	for (unsigned int i=1;i<4;i++) {
		if (bbplot[i].get_oX()<minx) minx=bbplot[i].get_oX();
		if (bbplot[i].get_oY()<miny) miny=bbplot[i].get_oY();
		if (bbplot[i].get_oX()>maxx) maxx=bbplot[i].get_oX();
		if (bbplot[i].get_oY()>maxy) maxy=bbplot[i].get_oY();
	}

	//Filter the point cloud and keep only those points inside the bounding box
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PassThrough<pcl::PointXYZRGB> passx;
	passx.setInputCloud (cloud_);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (minx, maxx);
	passx.filter (*cloud_filtered_x);

	pcl::PassThrough<pcl::PointXYZRGB> passy;
	passy.setInputCloud (cloud_filtered_x);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (miny, maxy);
	passy.filter (*cloud_filtered_xy);

	//Outlier removal
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	// build the filter
	outrem.setInputCloud(cloud_filtered_xy);
	outrem.setRadiusSearch(0.08);
	outrem.setMinNeighborsInRadius (10);

	// apply filter
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy_out (new pcl::PointCloud<pcl::PointXYZRGB>);
	ROS_INFO("Applying outlier removal filter...");
	outrem.filter (*cloud_filtered_xy_out);
	ROS_INFO("Done");

	//Downsampling the pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud_filtered_xy_out);
	sor.setLeafSize (0.04, 0.04, 0.04);
	sor.filter (*cloud_downsampled);

	//Find the maxz and floorz on the cloud
	double minz=10000, floorz=-10000;
	int minzPoint=0;
	for (unsigned int i=0; i<cloud_downsampled->points.size(); i++) {
		//for the moment, floorz will be minz. FIXME
		if (cloud_downsampled->points[i].z>floorz) floorz=cloud_downsampled->points[i].z;
		if (cloud_downsampled->points[i].z<minz) {
			minz=cloud_downsampled->points[i].z;
			minzPoint=i;
		}
	}
	ROS_INFO_STREAM("minz is: " << minz);
	ROS_INFO_STREAM("floorz is: " << floorz);
	ROS_INFO_STREAM("Object height is " << floorz-minz);
	double gf_penetration=(((floorz-minz)>grasp_penetration_) ? grasp_penetration_ : floorz-minz );
	ROS_INFO_STREAM("Grasp frame penetration is " <<  gf_penetration);

	//Setting the grasp frame
	//Z is minz+penetration
	//X,Y are X,Y of the minz point
	//Z axis is looking towards the penetration direction
	//X,Y axis are aligned with the bounding box axis
	cMg[0][3]=cloud_downsampled->points[minzPoint].x;
	cMg[1][3]=cloud_downsampled->points[minzPoint].y;
	cMg[2][3]=cloud_downsampled->points[minzPoint].z+gf_penetration;
	vpColVector n(3),o(3),a(3);
	a[0]=0; a[1]=0; a[2]=1;
	o=(ip2m-ip1m).normalize();
	n=vpColVector::cross(o,a);
	cMg[0][0]=n[0]; cMg[0][1]=o[0]; cMg[0][2]=a[0];
	cMg[1][0]=n[1]; cMg[1][1]=o[1]; cMg[1][2]=a[1];
	cMg[2][0]=n[2]; cMg[2][1]=o[2]; cMg[2][2]=a[2];
	ROS_INFO_STREAM("cMg is: " << std::endl << cMg);

	//Compute bMg and plan a grasp on bMg
	//vpHomogeneousMatrix bMg=bMc*cMg;
	//std::cerr << "bMg is: " << std::endl << bMg << std::endl;
}


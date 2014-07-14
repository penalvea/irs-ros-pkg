/*
 * VispUtils: usage example in http://github.com/davidfornas/pcl_manipulation
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#ifndef PCLUTILS_H_
#define PCLUTILS_H_

#include <tf/transform_broadcaster.h>
#include <visp/vpHomogeneousMatrix.h>
#include <string>
#include <visualization_msgs/Marker.h>


// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

//FILTERS
#include <pcl/filters/passthrough.h>

//SEG
#include <pcl/segmentation/sac_segmentation.h>

//Filters Downsampling
#include <pcl/filters/voxel_grid.h>
//sor outliers
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

typedef pcl::PointXYZRGB PointT;

namespace PCLUtils{
	
	
void passThrough(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, double, double);
void statisticalOutlierRemoval();
void voxelGridFilter();
void estimateNormals(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
pcl::ModelCoefficients::Ptr planeSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
											  pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr);

pcl::ModelCoefficients::Ptr cylinderSegmentation(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr,
											     pcl::PointCloud<PointT>::Ptr);
	/*

struct Frame{
	tf::Transform pose;
	std::string parent, child;
	friend std::ostream& operator<<(std::ostream& out, Frame& x );
};

std::ostream& operator<<(std::ostream&, Frame&);
tf::Transform tfTransFromVispHomog( vpHomogeneousMatrix);

class VispToTF
{

	tf::TransformBroadcaster *broadcaster_;
	std::map<std::string, Frame> frames_;

public:

    VispToTF( vpHomogeneousMatrix sMs, std::string parent, std::string child );
.... };*/


}

#endif

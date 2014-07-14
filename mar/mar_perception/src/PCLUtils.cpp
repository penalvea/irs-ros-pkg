/*
 * PCLUtils: usage example in http://github.com/davidfornas/pcl_manipulation
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#include <mar_perception/PCLUtils.h>

//...

namespace PCLUtils{
	
void passThrough(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out, double z_min, double z_max){
  pcl::PassThrough<PointT> pass;
  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10, 10);
  pass.filter (*out);
}

void statisticalOutlierRemoval(){
		
}

void voxelGridFilter(){
		
}

void estimateNormals(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){  
  
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());  
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (in);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
	
}

pcl::ModelCoefficients::Ptr planeSegmentation(pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals,
											  pcl::PointCloud<PointT>::Ptr out_cloud, pcl::PointCloud<pcl::Normal>::Ptr out_normals){

  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (in_cloud);
  seg.setInputNormals (in_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (in_cloud);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points. Saved." << std::endl;
  writer.write ("/home/david/scene_plane.pcd", *cloud_plane, false); 

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*out_cloud);
  
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (in_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*out_normals);

  return coefficients_plane;
}

pcl::ModelCoefficients::Ptr cylinderSegmentation(pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr in_normals,
											  pcl::PointCloud<PointT>::Ptr cloud_cylinder){

  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);  
  pcl::ExtractIndices<PointT> extract;
  pcl::PCDWriter writer;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (20000);//10000
  seg.setDistanceThreshold (0.05);//0.05
  seg.setRadiusLimits (0, 0.1);//0, 0.1
  seg.setInputCloud (in_cloud);
  seg.setInputNormals (in_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inlier1s to disk
  extract.setInputCloud (in_cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    std::cerr << *coefficients_cylinder << std::endl;
    writer.write ("/home/david/scene_cylinder.pcd", *cloud_cylinder, false);
  }

  return coefficients_cylinder;
}
	
}

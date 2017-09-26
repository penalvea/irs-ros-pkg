#ifndef CLOUD_REGISTRATION_H
#define CLOUD_REGISTRATION_H
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/search.h>
#include <pcl/filters/project_inliers.h>

#include <string>
#include <stdio.h>

class CloudRegistration
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr main_scene_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scenes_, scenes2_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int n_objects);
  std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  std::vector<pcl::PointIndices> segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  std::vector<pcl::PointIndices> segmentCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointCloud<pcl::PointNormal>::Ptr downsampleCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
  pcl::PointCloud<pcl::PointNormal>::Ptr alignCloud(pcl::PointCloud<pcl::PointNormal>::Ptr scene, pcl::PointCloud<pcl::PointNormal>::Ptr object);
  Eigen::Matrix4f alignCloud_transformation(pcl::PointCloud<pcl::PointNormal>::Ptr scene, pcl::PointCloud<pcl::PointNormal>::Ptr object);
  pcl::PointCloud<pcl::PointNormal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr addPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool main);

public:
  CloudRegistration();
  CloudRegistration(std::string main_scene_path, std::vector<std::string> scene_paths, std::vector<std::string> scene_paths2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void showCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
  void showClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr registerClouds(int n_objects);
  pcl::PointCloud<pcl::PointXYZ>::Ptr registerClouds_onebyone(int n_objects);

};

#endif // CLOUD_REGISTRATION_H

#ifndef BESTGRASP_H
#define BESTGRASP_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
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
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <string>
#include <stdio.h>


class BestGrasp
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr getObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  std::vector<pcl::PointIndices> segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
   void showCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void showClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
  pcl::PointXYZ linePlaneIntersection(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ line_p1, pcl::PointXYZ line_p2);
  pcl::ModelCoefficients::Ptr threePointsToPlane (pcl::PointXYZ point_a, pcl::PointXYZ point_b, pcl::PointXYZ point_c);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b);

public:
  BestGrasp();
  std::vector<pcl::PointXYZ> getBestGrasp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};




#endif // BESTGRASP_H

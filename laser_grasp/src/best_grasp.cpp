#include "laser_grasp/best_grasp.h"

BestGrasp::BestGrasp()
{}



void BestGrasp::showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  viewer.addPointCloud<pcl::PointXYZ>(cloud);
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  viewer.close();

}
void BestGrasp::showCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "color cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "color cloud");
  viewer.initCameraParameters ();
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  viewer.close();










}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BestGrasp::color_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud, *cloud_color);
  for(int i=0; i< cloud_color->width; i++){
    cloud_color->points[i].r=r;
    cloud_color->points[i].g=g;
    cloud_color->points[i].b=b;
  }
  showCloud(cloud_color);
  return cloud_color;
}

void BestGrasp::showClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud1");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud2, single_color2, "cloud2");
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BestGrasp::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting downsampling...\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  grid.filter (*cloud_downsampled);

  return cloud_downsampled;
}

std::vector<pcl::PointIndices> BestGrasp::segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting segmentation...\n");

  //showCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);


  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  ec.setClusterTolerance (0.01); // 1cm

  ec.setMinClusterSize (100);

  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);


  ec.extract (cluster_indices);

  for(int i=0; i<cluster_indices.size(); i++){
    std::cout<<"Cluster "<<cluster_indices[i].indices.size()<<std::endl;
  }



  return cluster_indices;
}


std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> BestGrasp::getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Getting the plane...\n");
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> pair(inliers, coefficients);

    return pair;
}


pcl::PointXYZ BestGrasp::linePlaneIntersection(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ line_p1, pcl::PointXYZ line_p2){
  /*line:
   * x_t=a*t+a0
   * y_t=b*t+b0
   * z_t=c*t+c0
   *
   * plane:
   * A*x_t+B*y_t+C*z_t+D=0
   *
   * A*(a*t+a0)+B*(b*t+b0)+C*(c*t+c0)+D=0
   *
   * t= -(A*a0+B*b0+C*c0+D)/(A*a+B*b+C*c)
   */

  float a_0=line_p1.x;
  float b_0=line_p1.y;
  float c_0=line_p1.z;

  float a=line_p2.x-line_p1.x;
  float b=line_p2.y-line_p1.y;
  float c=line_p2.z-line_p1.z;

  float A=plane->values[0];
  float B=plane->values[1];
  float C=plane->values[2];
  float D=plane->values[3];

  float t=-(A*a_0+B*b_0+C*c_0+D)/(A*a+B*b+C*c);

  pcl::PointXYZ point(a*t+a_0, b*t+b_0, c*t+c_0);
  return point;


}


pcl::ModelCoefficients::Ptr BestGrasp::threePointsToPlane (pcl::PointXYZ point_a, pcl::PointXYZ point_b, pcl::PointXYZ point_c){

  Eigen::Vector3d dir1(point_a.x-point_b.x, point_a.y-point_b.y, point_a.z-point_b.z);
  Eigen::Vector3d dir2(point_a.x-point_c.x, point_a.y-point_c.y, point_a.z-point_c.z);

  Eigen::Vector3d norm=dir1.cross(dir2);

  double d=-point_a.x*norm[0]-point_a.y*norm[1]-point_a.z*norm[2];

  pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
  plane->values.resize (4);

  plane->values[0]=norm[0];
  plane->values[1]=norm[1];
  plane->values[2]=norm[2];
  plane->values[3]=d;

  return plane;
}

std::vector<pcl::PointXYZ> BestGrasp::getBestGrasp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud=downsampleCloud(cloud);
  showCloud(downsampled_cloud);
  std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> plane=getPlane(downsampled_cloud);
  pcl::PointIndices::Ptr plane_inliers=plane.first;
  pcl::ModelCoefficients::Ptr coefficients=plane.second;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract(true);
  extract.setInputCloud(downsampled_cloud);
  extract.setIndices(plane_inliers);
  extract.setNegative(true);
  extract.filter(*cloud_no_plane);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_plane_color=color_cloud(cloud_no_plane, 0, 255, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::ExtractIndices<pcl::PointXYZ> extract(true);
  extract.setInputCloud(downsampled_cloud);
  extract.setIndices(plane_inliers);
  extract.setNegative(false);
  extract.filter(*cloud_plane);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_color=color_cloud(cloud_plane, 255, 0, 0);
  *cloud_plane_color+=*cloud_no_plane_color;
  showCloud(cloud_plane_color);

  std::vector<pcl::PointIndices> objects_indices=segmentCloud(cloud_no_plane);

  pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud_no_plane, *object);
  object->points.clear();

  for(int j=objects_indices[0].indices.size()-1; j>=0; j--){
    pcl::PointXYZ point(cloud_no_plane->points[objects_indices[0].indices[j]].x, cloud_no_plane->points[objects_indices[0].indices[j]].y, cloud_no_plane->points[objects_indices[0].indices[j]].z);
    object->points.push_back(point);
  }
  object->width=object->points.size();
  showCloud(object);

  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (object);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;

  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia (moment_of_inertia);

  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->addPointCloud<pcl::PointXYZ> (object, "sample cloud");

  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);


  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));


  pcl::PointXYZ center_plane_projection=linePlaneIntersection(coefficients, center, x_axis);
  double distance=std::sqrt(std::pow(center.x-center_plane_projection.x,2)+std::pow(center.y-center_plane_projection.y,2)+ std::pow(center.z-center_plane_projection.z,2));
  char axis_plane='x';
  pcl::PointXYZ center_plane_projection_aux=linePlaneIntersection(coefficients, center, y_axis);
  double distance_aux=std::sqrt(std::pow(center.x-center_plane_projection_aux.x,2)+std::pow(center.y-center_plane_projection_aux.y,2)+ std::pow(center.z-center_plane_projection_aux.z,2));
  if(distance_aux<distance){
    distance=distance_aux;
    center_plane_projection=center_plane_projection_aux;
    axis_plane='y';
  }
  center_plane_projection_aux=linePlaneIntersection(coefficients, center, z_axis);
  distance_aux=std::sqrt(std::pow(center.x-center_plane_projection_aux.x,2)+std::pow(center.y-center_plane_projection_aux.y,2)+ std::pow(center.z-center_plane_projection_aux.z,2));
  if(distance_aux<distance){
    distance=distance_aux;
    center_plane_projection=center_plane_projection_aux;
    axis_plane='z';
  }


  Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

  p1 = rotational_matrix_OBB * p1 + position;
  p2 = rotational_matrix_OBB * p2 + position;
  p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  p6 = rotational_matrix_OBB * p6 + position;
  p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;

  pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
  pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
  pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
  pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
  pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
  pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
  pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
  pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));





  pcl::PointXYZ center_bounding_projection=linePlaneIntersection(threePointsToPlane(pt2, pt3, pt6), center, center_plane_projection);

  pcl::PointXYZ new_center((center_bounding_projection.x+center_plane_projection.x)/2, (center_bounding_projection.y+center_plane_projection.y)/2, (center_bounding_projection.z+center_plane_projection.z)/2);



  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");



 /* viewer->addLine (center_plane_projection, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector2");
  viewer->addLine (center_plane_projection, y_axis, 1.0f, 0.0f, 0.0f, "middle eigen vector2");
  viewer->addLine (center_plane_projection, z_axis, 1.0f, 0.0f, 0.0f, "minor eigen vector2");

  viewer->addLine (center_bounding_projection, x_axis, 0.0f, 1.0f, 0.0f, "major eigen vector3");
  viewer->addLine (center_bounding_projection, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector3");
  viewer->addLine (center_bounding_projection, z_axis, 0.0f, 1.0f, 0.0f, "minor eigen vector3");*/






  pcl::PointXYZ new_pt1=linePlaneIntersection(coefficients, pt1, pt2);
  pcl::PointXYZ new_pt4=linePlaneIntersection(coefficients, pt4, pt3);
  pcl::PointXYZ new_pt5=linePlaneIntersection(coefficients, pt5, pt6);
  pcl::PointXYZ new_pt8=linePlaneIntersection(coefficients, pt8, pt7);







  viewer->addLine (pt1, pt2, 1.0, 1.0, 0.0, "1 edge");
  viewer->addLine (pt1, pt4, 1.0, 1.0, 0.0, "2 edge");
  viewer->addLine (pt1, pt5, 1.0, 1.0, 0.0, "3 edge");
  viewer->addLine (pt5, pt6, 1.0, 1.0, 0.0, "4 edge");
  viewer->addLine (pt5, pt8, 1.0, 1.0, 0.0, "5 edge");
  viewer->addLine (pt2, pt6, 1.0, 1.0, 0.0, "6 edge");
  viewer->addLine (pt6, pt7, 1.0, 1.0, 0.0, "7 edge");
  viewer->addLine (pt7, pt8, 1.0, 1.0, 0.0, "8 edge");
  viewer->addLine (pt2, pt3, 1.0, 1.0, 0.0, "9 edge");
  viewer->addLine (pt4, pt8, 1.0, 1.0, 0.0, "10 edge");
  viewer->addLine (pt3, pt4, 1.0, 1.0, 0.0, "11 edge");
  viewer->addLine (pt3, pt7, 1.0, 1.0, 0.0, "12 edge");


  /*viewer->addLine (pt1, new_pt1, 0.0, 1.0, 1.0, "new 1");
  viewer->addLine (pt4, new_pt4, 0.0, 1.0, 1.0, "new 4");
  viewer->addLine (pt5, new_pt5, 0.0, 1.0, 1.0, "new 5");
  viewer->addLine (pt8, new_pt8, 0.0, 1.0, 1.0, "new 8");

  viewer->addLine (new_pt1, new_pt4, 0.0, 1.0, 1.0, "new 1_");
  viewer->addLine (new_pt4, new_pt8, 0.0, 1.0, 1.0, "new 4_");
  viewer->addLine (new_pt8, new_pt5, 0.0, 1.0, 1.0, "new 5_");
  viewer->addLine (new_pt5, new_pt1, 0.0, 1.0, 1.0, "new 8_");*/










  pcl::PointXYZ point1, point2, point3, point4;



  pcl::PointXYZ new_x_axis (major_vector (0) + new_center.x, major_vector (1) + new_center.y, major_vector (2) + new_center.z);
  pcl::PointXYZ new_y_axis (middle_vector (0) + new_center.x, middle_vector (1) + new_center.y, middle_vector (2) + new_center.z);
  pcl::PointXYZ new_z_axis (minor_vector (0) + new_center.x, minor_vector (1) + new_center.y, minor_vector (2) + new_center.z);

  viewer->addLine (new_center, new_x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector4");
  viewer->addLine (new_center, new_y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector4");
  viewer->addLine (new_center, new_z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector4");
  if(axis_plane=='x'){
    //y axis
    point1=linePlaneIntersection(threePointsToPlane(pt1, pt2, pt4), new_center, y_axis);
    point2=linePlaneIntersection(threePointsToPlane(pt5, pt6, pt8), new_center, y_axis);

    //z axis
     point3=linePlaneIntersection(threePointsToPlane(pt1, pt4, pt5), new_center, z_axis);
    point4=linePlaneIntersection(threePointsToPlane(pt2, pt3, pt6), new_center, z_axis);
  }
  else if(axis_plane=='y'){
    //x axis
    point1=linePlaneIntersection(threePointsToPlane(pt1, pt2, pt5), new_center, x_axis);
    point2=linePlaneIntersection(threePointsToPlane(pt4, pt3, pt8), new_center, x_axis);

    //z axis
    point3=linePlaneIntersection(threePointsToPlane(pt1, pt4, pt5), new_center, z_axis);
    point4=linePlaneIntersection(threePointsToPlane(pt2, pt3, pt6), new_center, z_axis);
  }
  else{
    //x axis
    point1=linePlaneIntersection(threePointsToPlane(pt1, pt2, pt5), center, x_axis);
    point2=linePlaneIntersection(threePointsToPlane(pt4, pt3, pt8), center, x_axis);

    //y axis
    point3=linePlaneIntersection(threePointsToPlane(pt1, pt2, pt4), center, y_axis);
    point4=linePlaneIntersection(threePointsToPlane(pt5, pt6, pt8), center, y_axis);
  }

  //viewer->addLine (point1, point2, 1.0, 1.0, 0.0, "12 edgee");
  //7viewer->addLine (point3, point4, 0.0, 1.0, 1.0, "122 edgee");


  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  std::vector<pcl::PointXYZ> points;
  points.push_back(point1);
  points.push_back(point2);
  points.push_back(point3);
  points.push_back(point4);
  points.push_back(new_center);
  points.push_back(center);
  return points;
}

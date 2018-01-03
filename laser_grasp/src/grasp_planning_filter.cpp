#include "laser_grasp/grasp_planning_filter.h"



GraspPlanningFilter::GraspPlanningFilter(){}

void GraspPlanningFilter::showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  viewer.addPointCloud<pcl::PointXYZ>(cloud);
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  viewer.close();

}
void GraspPlanningFilter::showCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
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
void GraspPlanningFilter::showCloudObjectsPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices> objects_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane){
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  for(int i=0; i<4; i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_new);
    cloud_new->points.clear();
    for(int j=0; j<objects_indices[i].indices.size(); j++){
      cloud_new->points.push_back(cloud->points[objects_indices[i].indices[j]]);
    }
    cloud_new->width=cloud_new->points.size();
    clouds.push_back(cloud_new);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_color=color_cloud(cloud_plane, 255, 0, 0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object1=color_cloud(clouds[0], 0, 255, 0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object2=color_cloud(clouds[1], 0, 0, 255);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object3=color_cloud(clouds[2], 0, 255, 255);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object4=color_cloud(clouds[3], 255, 255, 0);

  *cloud_plane_color+=*object1;
  *cloud_plane_color+=*object2;
  *cloud_plane_color+=*object3;
  *cloud_plane_color+=*object4;
  showCloud(cloud_plane_color);






}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GraspPlanningFilter::color_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud, *cloud_color);
  for(int i=0; i< cloud_color->width; i++){
    cloud_color->points[i].r=r;
    cloud_color->points[i].g=g;
    cloud_color->points[i].b=b;
  }
  //showCloud(cloud_color);
  return cloud_color;
}

void GraspPlanningFilter::showClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2){
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

pcl::PointCloud<pcl::PointXYZ>::Ptr GraspPlanningFilter::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting downsampling...\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  grid.filter (*cloud_downsampled);

  return cloud_downsampled;
}

std::vector<pcl::PointIndices> GraspPlanningFilter::segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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


std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> GraspPlanningFilter::getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Getting the plane...\n");
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.008);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> pair(inliers, coefficients);

  return pair;
}


pcl::PointXYZ GraspPlanningFilter::linePlaneIntersection(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ line_p1, pcl::PointXYZ line_p2){
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


pcl::ModelCoefficients::Ptr GraspPlanningFilter::threePointsToPlane (pcl::PointXYZ point_a, pcl::PointXYZ point_b, pcl::PointXYZ point_c){

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

float GraspPlanningFilter::distPoint2Plane(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr plane){
  float lambda=(plane->values[0] * point.x + plane->values[1] * point.y + plane->values[2] * point.z + plane->values[3]) / -(std::pow(plane->values[0],2) + std::pow(plane->values[1],2) + std::pow(plane->values[2],2));

  pcl::PointXYZ point_plane;
  point_plane.x=point.x+ lambda*plane->values[0];
  point_plane.y=point.y+ lambda*plane->values[1];
  point_plane.z=point.z+ lambda*plane->values[2];


  return distPoints(point, point_plane);
}

float GraspPlanningFilter::distPoints(pcl::PointXYZ point1, pcl::PointXYZ point2){
  return std::sqrt(std::pow(point1.x-point2.x,2)+std::pow(point1.y-point2.y,2)+std::pow(point1.z-point2.z,2));
}

pcl::PointXYZ GraspPlanningFilter::projectionPlaneLine(pcl::PointXYZ point_center, Eigen::Vector3f dir, pcl::ModelCoefficients::Ptr plane){
  pcl::PointXYZ point2;
  point2.x=point_center.x+(2*dir(0));
  point2.y=point_center.y+(2*dir(1));
  point2.z=point_center.z+(2*dir(2));

  pcl::PointXYZ point3;
  point3.x=point2.x+(2*plane->values[0]);
  point3.y=point2.y+(2*plane->values[1]);
  point3.z=point2.z+(2*plane->values[2]);


  return linePlaneIntersection(plane, point2, point3);

}

pcl::PointXYZ GraspPlanningFilter::projectionPlanePoint(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr plane){
  pcl::PointXYZ point2;
  point2.x=point.x+(2*plane->values[0]);
  point2.y=point.y+(2*plane->values[1]);
  point2.z=point.z+(2*plane->values[2]);

  return linePlaneIntersection(plane, point, point2);
}


pcl::PointXYZ GraspPlanningFilter::pointAtDistance(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr plane, float distance){
  float lambda=distance/std::sqrt(std::pow(plane->values[0],2)+ std::pow(plane->values[1], 2)+ std::pow(plane->values[2],2));
  pcl::PointXYZ point2(point.x-lambda*plane->values[0], point.y-lambda*plane->values[1], point.z-lambda*plane->values[2]);
  return point2;
}

float GraspPlanningFilter::angleVectors(Eigen::Vector3f vec1, Eigen::Vector3f vec2){
  float angle=std::abs(std::acos(vec1(0)*vec2(0)+vec1(1)*vec2(1)+vec1(2)*vec2(2)));
  if(angle>3.14)
    angle-=3.14;

}



task_priority::ObjectPose_msg GraspPlanningFilter::publishPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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

  showCloudObjectsPlane(cloud_no_plane, objects_indices, cloud_plane);

  pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects_projected;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects_updown;
  for(int cont=0; cont<4; cont++){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_no_plane, *cloud_points_projected);
    cloud_points_projected->points.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_updown(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_no_plane, *cloud_points_updown);
    cloud_points_updown->points.clear();



    float max_dist=0;

    for(int j=objects_indices[cont].indices.size()-1; j>=0; j--){
      pcl::PointXYZ point(cloud_no_plane->points[objects_indices[cont].indices[j]].x, cloud_no_plane->points[objects_indices[cont].indices[j]].y, cloud_no_plane->points[objects_indices[cont].indices[j]].z);
      pcl::PointXYZ point_projected=projectionPlanePoint(point, coefficients);
      if(distPoints(point, point_projected)>max_dist){
        max_dist=distPoints(point, point_projected);
      }
    }

    for(int j=objects_indices[cont].indices.size()-1; j>=0; j--){
      pcl::PointXYZ point(cloud_no_plane->points[objects_indices[cont].indices[j]].x, cloud_no_plane->points[objects_indices[cont].indices[j]].y, cloud_no_plane->points[objects_indices[cont].indices[j]].z);

      cloud_points_projected->points.push_back(point);
      pcl::PointXYZ projected=projectionPlanePoint(point, coefficients);
      cloud_points_projected->points.push_back(projected);
      cloud_points_updown->points.push_back(projected);
      cloud_points_updown->points.push_back(pointAtDistance(projected, coefficients, max_dist));

    }
    cloud_points_projected->width=cloud_points_projected->points.size();
    //showCloud(cloud_points_projected);
    //showCloud(cloud_points_updown);
    objects_projected.push_back(cloud_points_projected);
    objects_updown.push_back((cloud_points_updown));
  }


task_priority::ObjectPose_msg msg;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  for(int i=0; i<1; i++){


  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (objects_updown[i]);
  feature_extractor.compute ();

  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  Eigen::Vector3f mass_center;

  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getMassCenter (mass_center);

  pcl::PointXYZ point_center;
  point_center.x=(max_point_OBB.x+min_point_OBB.x)/2+position_OBB.x;
  point_center.y=(max_point_OBB.y+min_point_OBB.y)/2+position_OBB.y;
  point_center.z=(max_point_OBB.z+min_point_OBB.z)/2+position_OBB.z;


   std::ostringstream ss;
   ss<<i;
   std::string cloud_name="cloud_"+ss.str();
   std::string cube_name="cube_"+ss.str();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color;
   if(i==0){
     color=color_cloud(objects_projected[i], 255, 255, 255);
   }
   else if(i==1){
     color=color_cloud(objects_projected[i], 0, 0, 255);
   }
   else if(i==2){
     color=color_cloud(objects_projected[i], 0, 255, 255);
   }
   else if(i==3){
     color=color_cloud(objects_projected[i], 255, 255, 0);
   }
   else{
     color=color_cloud(objects_projected[i], 255, 0, 0);
   }

   //pcl::PointCloud<pcl::PointXYZRGB>::Ptr color=color_cloud(objects_projected[i], 255, 0, 0);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(color);



    viewer->addPointCloud<pcl::PointXYZRGB> (color,rgb, cloud_name);
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, cube_name);



    float x_offset=max_point_OBB.x - min_point_OBB.x;
    float y_offset=max_point_OBB.y - min_point_OBB.y;
    float z_offset=max_point_OBB.z - min_point_OBB.z;

    Eigen::Matrix3f rot;

    if(x_offset>y_offset && x_offset>z_offset){
      std::cout<<"Entro1"<<std::endl;
      rot=Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX());
      std::cout<<rot<<std::endl;
      rot=rot*Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());
      std::cout<<Eigen::Vector3f::UnitZ()<<std::endl;
    msg.x_offset=z_offset;
    msg.y_offset=x_offset;
    msg.z_offset=y_offset;
    std::cout<<rot<<std::endl;
    }
    else if(y_offset>z_offset){
      std::cout<<"Entro2"<<std::endl;
      rot=Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY());
      msg.x_offset=z_offset;
      msg.y_offset=y_offset;
      msg.z_offset=x_offset;
    }
    else{
      std::cout<<"Entro3"<<std::endl;

      rot=Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY());
      rot=rot*Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
      msg.x_offset=y_offset;
      msg.y_offset=z_offset;
      msg.z_offset=x_offset;
    }
    Eigen::Matrix3f mat(quat*Eigen::Quaternionf(rot));
    //viewer->addCube (position, quat*Eigen::Quaternionf(rot), msg.x_offset, msg.y_offset, msg.z_offset, "OBB");
    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));



      /*pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
      pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
      pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));*/


        pcl::PointXYZ x_axis (mat(0,0) + mass_center (0), mat(1,0) + mass_center (1), mat(2,0) + mass_center (2));
        pcl::PointXYZ y_axis (mat(0,1) + mass_center (0), mat(1,1) + mass_center (1), mat(2,1) + mass_center (2));
        pcl::PointXYZ z_axis (mat(0,2) + mass_center (0), mat(1,2) + mass_center (1), mat(2,2) + mass_center (2));
        viewer->addLine (center, x_axis, 0.0f, 1.0f, 0.0f, "major eigen vector");
        viewer->addLine (center, y_axis, 1.0f, 0.0f, 0.0f, "middle eigen vector");
        viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

        /*viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
          viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
          viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");*/

    /*Eigen::Vector3f x_robot(1,0,0), y_robot(0,1,0), z_robot(0,0,1);

    Eigen::Vector3f x_object, y_object, z_object;

    Eigen::Vector3f positive, negative;
    if(angleVectors(z_robot, major_vector)< angleVectors(z_robot, middle_vector)  && angleVectors(z_robot, major_vector)< angleVectors(z_robot, minor_vector)){
      std::cout<<"Objeto Posicionado verticalmente"<<std::endl;
      return;
    }
    else{
      pcl::PointXYZ proj=projectionPlanePoint(pcl::PointXYZ(mass_center(0), mass_center(1), mass_center(2)), coefficients);
      Eigen::Vector3f proj_e(proj.x, proj.y, proj.z);
      z_object=proj_e-mass_center;
      z_object.normalize();*/

      /*if(angleVectors(z_robot, minor_vector)< angleVectors(z_robot, middle_vector)){
        positive=mass_center+minor_vector;
        negative=mass_center-minor_vector;
        if(positive.norm()>negative.norm())
          z_object=minor_vector;
        else
          z_object=-minor_vector;
      }
      else{
        positive=mass_center+middle_vector;
        negative=mass_center-middle_vector;
        if(positive.norm()>negative.norm())
          z_object=middle_vector;
        else
          z_object=-middle_vector;
      }*/

      /*positive=mass_center+middle_vector;
      negative=mass_center-middle_vector;
      if(positive.norm()>negative.norm())
        y_object=middle_vector;
      else
        y_object=-middle_vector;
     x_object=z_object.cross(y_object);
    }

    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    pcl::PointXYZ x_axis (x_object (0) + mass_center (0), x_object (1) + mass_center (1), x_object (2) + mass_center (2));
    pcl::PointXYZ y_axis (y_object (0) + mass_center (0), y_object (1) + mass_center (1), y_object (2) + mass_center (2));
    pcl::PointXYZ z_axis (z_object (0) + mass_center (0), z_object (1) + mass_center (1), z_object (2) + mass_center (2));
    viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");*/









    Eigen::Vector3f y_ax;
    y_ax(0)=major_vector (0);
    y_ax(1)=major_vector (1);
    y_ax(2)=major_vector (2);
    y_ax.normalize();




  msg.center.position.x=mass_center(0);
  msg.center.position.y=mass_center(1);
  msg.center.position.z=mass_center(2);
  geometry_msgs::Quaternion q_msg;

  Eigen::Quaternionf q=Eigen::Quaternionf(mat);
  q_msg.x=q.x();
  q_msg.y=q.y();
  q_msg.z=q.z();
  q_msg.w=q.w();
  msg.center.orientation=q_msg;

  }
  while(!viewer->wasStopped())
   {
     viewer->spinOnce (100);
     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }

  return msg;




   /* Eigen::Vector3f x_plane;
    x_plane(0)=point_dir_plane_rotation.x-point_center_plane.x;
    x_plane(1)=point_dir_plane_rotation.y-point_center_plane.y;
    x_plane(2)=point_dir_plane_rotation.z-point_center_plane.z;
    x_plane.normalize();

    Eigen::Vector3f y_plane, z_plane;

    z_plane(0)=coefficients_new_plane->values[0];
    z_plane(1)=coefficients_new_plane->values[1];
    z_plane(2)=coefficients_new_plane->values[2];

    z_plane.normalize();

    y_plane=x_plane.cross(z_plane);
    y_plane.normalize();


    Eigen::Matrix4f origin_to_rotation_center;
    origin_to_rotation_center<<x_plane(0), y_plane(0), z_plane(0), point_center_plane.x, x_plane(1), y_plane(1), z_plane(1), point_center_plane.y, x_plane(2), y_plane(2), z_plane(2), point_center_plane.z, 0, 0, 0, 1;
    std::cout<<"origin_to_rotation_center"<<std::endl<<origin_to_rotation_center<<std::endl;
*/






    /*pcl::visualization::PCLVisualizer viewer("Cloud");
    viewer.addPointCloud<pcl::PointXYZ>(cloud);
    viewer.addPlane(*coefficients_new_plane, "plane1");
    viewer.addPlane(*best_plane, point_center_plane.x, point_center_plane.y, point_center_plane.z, "plane2");

    //viewer.addLine(point_center.x-0.1, point_center.y, point_center.z, point_center.x+0.1, point_center.y, point_center.z, 1.0, 0.0, 0.0);
    viewer.addSphere(point_center_plane, 0.005, 1.0, 0.0, 0.0);



    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
    viewer.close();*/





}

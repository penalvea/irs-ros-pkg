#include "laser_grasp/symmetry.h"



Symmetry::Symmetry(){}


void Symmetry::showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::visualization::PCLVisualizer viewer("Cloud");
  viewer.addPointCloud<pcl::PointXYZ>(cloud);
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  viewer.close();

}
void Symmetry::showCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Symmetry::color_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b){
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

void Symmetry::showClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2){
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

pcl::PointCloud<pcl::PointXYZ>::Ptr Symmetry::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight ("Starting downsampling...\n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (cloud);
  grid.filter (*cloud_downsampled);

  return cloud_downsampled;
}

std::vector<pcl::PointIndices> Symmetry::segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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


std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> Symmetry::getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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


pcl::PointXYZ Symmetry::linePlaneIntersection(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ line_p1, pcl::PointXYZ line_p2){
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


pcl::ModelCoefficients::Ptr Symmetry::threePointsToPlane (pcl::PointXYZ point_a, pcl::PointXYZ point_b, pcl::PointXYZ point_c){

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

float Symmetry::distPoint2Plane(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr plane){
  float lambda=(plane->values[0] * point.x + plane->values[1] * point.y + plane->values[2] * point.z + plane->values[3]) / -(std::pow(plane->values[0],2) + std::pow(plane->values[1],2) + std::pow(plane->values[2],2));

  pcl::PointXYZ point_plane;
  point_plane.x=point.x+ lambda*plane->values[0];
  point_plane.y=point.y+ lambda*plane->values[1];
  point_plane.z=point.z+ lambda*plane->values[2];


  return distPoints(point, point_plane);

}
float Symmetry::distPoints(pcl::PointXYZ point1, pcl::PointXYZ point2){
  return std::sqrt(std::pow(point1.x-point2.x,2)+std::pow(point1.y-point2.y,2)+std::pow(point1.z-point2.z,2));
}

pcl::PointXYZ Symmetry::projectionPlaneLine(pcl::PointXYZ point_center, Eigen::Vector3f dir, pcl::ModelCoefficients::Ptr plane){
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

pcl::PointXYZ Symmetry::projectionPlanePoint(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr plane){
  pcl::PointXYZ point2;
  point2.x=point.x+(2*plane->values[0]);
  point2.y=point.y+(2*plane->values[1]);
  point2.z=point.z+(2*plane->values[2]);

  return linePlaneIntersection(plane, point, point2);
}

void Symmetry::calculateSymmetry(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
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

  float max_dist=0;
  pcl::PointXYZ point_max_dist;
  for(int j=objects_indices[0].indices.size()-1; j>=0; j--){
    pcl::PointXYZ point(cloud_no_plane->points[objects_indices[0].indices[j]].x, cloud_no_plane->points[objects_indices[0].indices[j]].y, cloud_no_plane->points[objects_indices[0].indices[j]].z);
    float dist=distPoint2Plane(point, coefficients);
    if(dist>max_dist){
      max_dist=dist;
      point_max_dist=point;
    }
    object->points.push_back(point);
  }
  object->width=object->points.size();
  std::cout<<"Punto mas lejano del plano "<<max_dist<<std::endl;




  ///////////////////////////////////
  ///
  ///
  ///

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aux(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*object, *aux);
  for(int i=0; i<aux->points.size(); i++){
    if(aux->points[i].x== point_max_dist.x && aux->points[i].y == point_max_dist.y && aux->points[i].z == point_max_dist.z){
      aux->points[i].r=255;
      aux->points[i].g=0;
      aux->points[i].b=0;
    }
    else{
      aux->points[i].r=0;
      aux->points[i].g=255;
      aux->points[i].b=0;
    }
  }

  /* pcl::visualization::PCLVisualizer viewer("Cloud");
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(aux);
  viewer.addPointCloud<pcl::PointXYZRGB> (aux, rgb, "color cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "color cloud");
  viewer.initCameraParameters ();
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  viewer.close();*/


  ///
  /// //////////////////////



  //translate plane to the middle of the object

  float lambda=(max_dist/2)/std::sqrt(std::pow(coefficients->values[0],2)+std::pow(coefficients->values[1],2)+std::pow(coefficients->values[2],2));
  std::cout<<"lambda= "<<lambda<<std::endl;

  pcl::PointXYZ point_plane;
  point_plane.x=-coefficients->values[3]/coefficients->values[0];
  point_plane.y=0;
  point_plane.z=0;

  pcl::PointXYZ point_new_plane;
  point_new_plane.x=point_plane.x+lambda*coefficients->values[0];
  point_new_plane.y=point_plane.y+lambda*coefficients->values[1];
  point_new_plane.z=point_plane.z+lambda*coefficients->values[2];

  if(distPoints(point_max_dist, point_plane)<distPoints(point_max_dist, point_new_plane)){
    point_new_plane.x=point_plane.x-lambda*coefficients->values[0];
    point_new_plane.y=point_plane.y-lambda*coefficients->values[1];
    point_new_plane.z=point_plane.z-lambda*coefficients->values[2];
  }
  std::cout<<"llego a crear coeficients"<<std::endl;
  pcl::ModelCoefficients::Ptr coefficients_new_plane=boost::make_shared<pcl::ModelCoefficients>(*coefficients);
  coefficients_new_plane->values[0]=coefficients->values[0];
  std::cout<<"llego"<<std::endl;
  coefficients_new_plane->values[1]=coefficients->values[1];
  coefficients_new_plane->values[2]=coefficients->values[2];
  coefficients_new_plane->values[3]=-(point_new_plane.x*coefficients->values[0]+point_new_plane.y*coefficients->values[1]+point_new_plane.z*coefficients->values[2]);
  std::cout<<coefficients_new_plane->values[0]<<" "<<coefficients_new_plane->values[1]<<" "<<coefficients_new_plane->values[2]<<" "<<coefficients_new_plane->values[3]<<std::endl;




  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (object);
  feature_extractor.compute ();

  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

  pcl::PointXYZ point_center;
  point_center.x=(max_point_OBB.x+min_point_OBB.x)/2+position_OBB.x;
  point_center.y=(max_point_OBB.y+min_point_OBB.y)/2+position_OBB.y;
  point_center.z=(max_point_OBB.z+min_point_OBB.z)/2+position_OBB.z;

  pcl::PointXYZ point_center_plane=projectionPlanePoint(point_center, coefficients_new_plane);
  pcl::PointXYZ point_center_plane_aux=point_center_plane;


  std::cout<<"Llego al algoritmo para cojer todos los puntos"<<std::endl;
float maxi_x=-1000, maxi_y=-1000, maxi_z=-1000, mini_x=1000, mini_y=1000, mini_z=1000;
for(int i=0; i<object->points.size(); i++){
  if(object->points[i].x>maxi_x)
    maxi_x=object->points[i].x;
  if(object->points[i].y>maxi_y)
    maxi_y=object->points[i].y;
  if(object->points[i].z>maxi_z)
    maxi_z=object->points[i].z;
  if(object->points[i].x<mini_x)
    mini_x=object->points[i].x;
  if(object->points[i].y<mini_y)
    mini_y=object->points[i].y;
  if(object->points[i].z<mini_z)
    mini_z=object->points[i].z;
}
std::cout<<maxi_x<<" "<<maxi_y<<" "<<maxi_z<<" "<<mini_x<<" "<<mini_y<<" "<<mini_z<<std::endl;

float offset=0.01;
maxi_x+=offset;
maxi_y+=offset;
maxi_z+=offset;
mini_x-=offset;
mini_y-=offset;
mini_z-=offset;

for(int i=0; i<cloud->points.size(); i++){
 if(cloud->points[i].x<maxi_x && cloud->points[i].x>mini_x && cloud->points[i].y<maxi_y && cloud->points[i].y>mini_y && cloud->points[i].z<maxi_z && cloud->points[i].z>mini_z){

   object->points.push_back(cloud->points[i]);
 }
}
std::cout<<"nuvo objeto creado"<<std::endl;

/*  for(int i=0; i<500; i++){
    float voxelSize=0.001;
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(voxelSize);
    pcl::KdTree::
    octree.setInputCloud(object);
    for(int j=0; j<cloud->points.size(); j++){
      //completar con radius search
        if(octree.radiusSearch()){
          std::cout<<"entro"<<std::endl;
          object->points.push_back(cloud->points[j]);
        }
    }
    object->width=object->points.size();
    std::cout<<i<<std::endl;
  }
*/


  object->width=object->points.size();
  showCloud(object);
  //showClouds(object, cloud);

  for(float i=-0.01; i<0.01; i+=0.002){
    pcl::PointXYZ normal_new;

    normal_new.x=coefficients_new_plane->values[0]*i;
    normal_new.y=coefficients_new_plane->values[1]*i;
    normal_new.z=coefficients_new_plane->values[2]*i;
    point_center_plane.x=point_center_plane_aux.x+normal_new.x;
    point_center_plane.y=point_center_plane_aux.y+normal_new.y;
    point_center_plane.z=point_center_plane_aux.z+normal_new.z;


    pcl::PointXYZ point_dir_plane_rotation=projectionPlaneLine(point_center_plane, major_vector, coefficients_new_plane);














    Eigen::Vector3f x_plane;
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


    float angle_max;
    int max_side=0;
    bool best_front=true;
    pcl::ModelCoefficients::Ptr best_plane;
    for(float angle=0; angle<3.14; angle+=0.01){

      Eigen::Transform<float, 3, Eigen::Affine> rotation(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()));

      Eigen::Vector4f point_x, point_y;
      point_x<<0.01,0,0,1;
      point_y<<0,0.01,0,1;
      Eigen::Vector4f  point_y_rotated;
      point_y_rotated=rotation*point_y;



      Eigen::Vector4f x_final, y_final;
      x_final=origin_to_rotation_center*point_x;
      y_final=origin_to_rotation_center*point_y_rotated;


      pcl::PointXYZ pointX(x_final(0), x_final(1), x_final(2)), pointY(y_final(0), y_final(1), y_final(2));
      pcl::ModelCoefficients::Ptr plane_rotated=threePointsToPlane(point_center_plane, pointX, pointY);



      pcl::PointXYZ normal;
      normal.x=plane_rotated->values[0];
      normal.y=plane_rotated->values[1];
      normal.z=plane_rotated->values[2];
      pcl::PointXYZ point_normal, point_opp_normal;
      point_normal.x=point_center_plane.x+normal.x;
      point_normal.y=point_center_plane.y+normal.y;
      point_normal.z=point_center_plane.z+normal.z;
      point_opp_normal.x=point_center_plane.x-normal.x;
      point_opp_normal.y=point_center_plane.y-normal.y;
      point_opp_normal.z=point_center_plane.z-normal.z;

      int front=0, back=0;
      for(int i=0; i<object->points.size(); i++){
        pcl::PointXYZ p_front_aux;
        p_front_aux.x=point_normal.x-object->points[i].x;
        p_front_aux.y=point_normal.y-object->points[i].y;
        p_front_aux.z=point_normal.z-object->points[i].z;
        float dist_front=std::sqrt(std::pow(p_front_aux.x,2)+std::pow(p_front_aux.y,2)+std::pow(p_front_aux.z,2));

        pcl::PointXYZ p_back_aux;
        p_back_aux.x=point_opp_normal.x-object->points[i].x;
        p_back_aux.y=point_opp_normal.y-object->points[i].y;
        p_back_aux.z=point_opp_normal.z-object->points[i].z;
        float dist_back=std::sqrt(std::pow(p_back_aux.x,2)+std::pow(p_back_aux.y,2)+std::pow(p_back_aux.z,2));

        if(dist_front<dist_back){
          front++;
        }
        else{
          back++;
        }
      }

      if(front>=back){
        if(front> max_side){
          max_side=front;
          angle_max=angle;
          best_plane=plane_rotated;
          best_front=true;
        }
      }
      else{
        if(back>max_side){
          max_side=back;
          angle_max=angle;
          best_plane=plane_rotated;
          best_front=false;
        }
      }
      //std::cout<<"angle "<<angle<<": Front: "<<front<<"   Back: "<<back<<std::endl;

    }

    std::cout<<"Best angle: "<<angle_max <<" better side "<<best_front<<std::endl;


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











    pcl::PointCloud<pcl::PointXYZ>::Ptr symmetric_object(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ normal;
    normal.x=best_plane->values[0];
    normal.y=best_plane->values[1];
    normal.z=best_plane->values[2];
    pcl::PointXYZ point_normal, point_opp_normal;
    point_normal.x=point_center_plane.x+normal.x;
    point_normal.y=point_center_plane.y+normal.y;
    point_normal.z=point_center_plane.z+normal.z;
    point_opp_normal.x=point_center_plane.x-normal.x;
    point_opp_normal.y=point_center_plane.y-normal.y;
    point_opp_normal.z=point_center_plane.z-normal.z;

    for(int i=0; i<object->points.size(); i++){
      pcl::PointXYZ p_front_aux;
      p_front_aux.x=point_normal.x-object->points[i].x;
      p_front_aux.y=point_normal.y-object->points[i].y;
      p_front_aux.z=point_normal.z-object->points[i].z;
      float dist_front=std::sqrt(std::pow(p_front_aux.x,2)+std::pow(p_front_aux.y,2)+std::pow(p_front_aux.z,2));

      pcl::PointXYZ p_back_aux;
      p_back_aux.x=point_opp_normal.x-object->points[i].x;
      p_back_aux.y=point_opp_normal.y-object->points[i].y;
      p_back_aux.z=point_opp_normal.z-object->points[i].z;
      float dist_back=std::sqrt(std::pow(p_back_aux.x,2)+std::pow(p_back_aux.y,2)+std::pow(p_back_aux.z,2));

      if((dist_front<dist_back && best_front) || (dist_front>dist_back && !best_front)){

        pcl::PointXYZ symmetric_point;
        pcl::PointXYZ proj=projectionPlanePoint(object->points[i], best_plane);
        symmetric_point.x=proj.x-(object->points[i].x-proj.x);
        symmetric_point.y=proj.y-(object->points[i].y-proj.y);
        symmetric_point.z=proj.z-(object->points[i].z-proj.z);

        symmetric_object->points.push_back(object->points[i]);
        symmetric_object->points.push_back(symmetric_point);

      }
    }
    symmetric_object->width=symmetric_object->points.size();
    symmetric_object->height=1;



    pcl::visualization::PCLVisualizer viewer2("Cloud2");
    viewer2.addPointCloud<pcl::PointXYZ>(symmetric_object);

    viewer2.addPlane(*best_plane, point_center_plane.x, point_center_plane.y, point_center_plane.z, "plane2");

    //viewer.addLine(point_center.x-0.1, point_center.y, point_center.z, point_center.x+0.1, point_center.y, point_center.z, 1.0, 0.0, 0.0);
    viewer2.addSphere(point_center_plane, 0.005, 1.0, 0.0, 0.0);



    while (!viewer2.wasStopped ())
    {
      viewer2.spinOnce ();
    }
    viewer2.close();

  }

  return;
}

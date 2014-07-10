/*
 * PCGraspPlanning.cpp
 *
 *  Created on: 07/06/2012
 *      Author: dfornas
 *      Modified 23/01/2013 by dfornas
 */

#include <ros/ros.h>

//#include <mar_perception/VispUtils.h>
#include <mar_perception/PCAutonomousGraspPlanning.h>

#include <visp/vpPixelMeterConversion.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>

//Includes del reconstruction, algunos pueden sobrar...
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//Comprobar si son necesarias
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>

#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>

#include <vector>
#include <algorithm>

void PCAutonomousGraspPlanning::perceive() {

  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new  pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  //Nubes del plano y cilindro.
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ()),  cloud_cylinder (new pcl::PointCloud<PointT> ());

  // Read in the cloud data
  std::cerr << "PointCloud has: " << cloud_->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud_);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-10, 10);
  //pass.setFilterLimits (-2, 0.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // @todo : Add more filters -> downsampling and radial ooutlier removal.

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (20000);//10000
  seg.setDistanceThreshold (0.05);//0.05
  seg.setRadiusLimits (0, 0.1);//0, 0.1
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inlier1s to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    std::cerr << *coefficients_cylinder << std::endl;
  }

  //Calculate CYLINDER SHAPE

  //Puntos de agarre
  PointT mean,mean2;

  PointT max, min;  //Puntos que definiran el cilindro.
  PointT axis_point;
  //Punto de origen y dirección del cilindro.
  axis_point.x=coefficients_cylinder->values[0];
  axis_point.y=coefficients_cylinder->values[1];
  axis_point.z=coefficients_cylinder->values[2];
  tf::Vector3 normal(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
  ///tf::Transform t();
  ///MarkerPublisher mp();
  

  //Perpendicular cylinder.
  pcl::ModelCoefficients::Ptr coefficients_perpendicular (new pcl::ModelCoefficients(*coefficients_cylinder));
  //Cambio de dirección para una perpendicular cualquiera. En realidad tomo la que está en el plano Z, lo que me facilitará las cosas luego.
  double old_three=coefficients_perpendicular->values[3];
  coefficients_perpendicular->values[3]=coefficients_perpendicular->values[4];
  coefficients_perpendicular->values[4]=-old_three;
  coefficients_perpendicular->values[5]=0;
  //Unitario.
  double module=sqrt(pow(coefficients_perpendicular->values[3],2)+pow(coefficients_perpendicular->values[4],2)+pow(coefficients_perpendicular->values[5],2));
  coefficients_perpendicular->values[3]/=module;coefficients_perpendicular->values[4]/=module;coefficients_perpendicular->values[5]/=module;

  getMinMax3DAlongAxis(cloud_cylinder, &max, &min, axis_point, &normal);
  //Punto medio.
  mean.x=(max.x+min.x)/2;mean.y=(max.y+min.y)/2;mean.z=(max.z+min.z)/2;
  PointT middle(mean);
  //punto medio desplazado vector unitario perpendicular por el radio
  mean.x+=coefficients_perpendicular->values[3]*coefficients_perpendicular->values[6];
  mean.y+=coefficients_perpendicular->values[4]*coefficients_perpendicular->values[6];
  mean.z+=coefficients_perpendicular->values[5]*coefficients_perpendicular->values[6];

  mean2.x=mean.x+2*(middle.x-mean.x);
  mean2.y=mean.y+2*(middle.y-mean.y);
  mean2.z=mean.z+2*(middle.z-mean.z);

  vpPoint g1, g2;
  g1.set_X(mean.x);g1.set_Y(mean.y);g1.set_Z(mean.z);
  g2.set_X(mean2.x);g2.set_Y(mean2.y);g2.set_Z(mean2.z);

  /*ROS_INFO_STREAM( "Grasping points are: " << std::endl 
    << "x:" << g1.get_X() << "y:" << g1.get_Y()  << "z:" << g1.get_Z()
    << "x:" << g2.get_X() << "y:" << g2.get_Y()  << "z:" << g2.get_Z()  << std::endl);
   */

  //DF: Two grasp points version.
  vpColVector ip1m(3), ip2m(3), middle_point(3),b(3);
  ip1m[0]=g1.get_X();ip1m[1]=g1.get_Y();ip1m[2]=g1.get_Z();
  ip2m[0]=g2.get_X();ip2m[1]=g2.get_Y();ip2m[2]=g2.get_Z();

  middle_point=0.5*(ip1m+ip2m);

  // Ahora mismo lo hace respecto del centro faltaría alejarlo R(radio)
  //Es decir, ahora mismo el end-efector cae dentro del cilindro en vez de en superfície.
  //Esto está relativamente bien pero no tenemos en cuenta la penetración. Sin emargo, la 
  //tenemos en cuenta luego al separanos el radio así que no hay problema en realidad. 
  base_cMg[0][3]=middle_point[0];
  base_cMg[1][3]=middle_point[1];
  base_cMg[2][3]=middle_point[2];//cloud_downsampled->points[minzPoint].z+gf_penetration;
  vpColVector n(3),o(3),a(3);

  //Este es el vector que el utiliza como la Z de la cámara. Si queremos rotar antes habría que cambiar este pero es más sencillo rotar luego.
  a[0]=0; a[1]=0; a[2]=1;
  n=(ip1m-ip2m).normalize();
  /*ROS_INFO_STREAM("Dot product is...: " << std::endl << vpColVector::dotProd(a,n) << std::endl);
    ROS_INFO_STREAM("N is...: " << std::endl << n << std::endl);
    ROS_INFO_STREAM("diff is...: " << std::endl << ip1m-ip2m << std::endl);
    ROS_INFO_STREAM("ip1m is...: " << std::endl << ip1m << std::endl);*/

  o=vpColVector::cross(a,n);
  base_cMg[0][0]=n[0]; base_cMg[0][1]=o[0]; base_cMg[0][2]=a[0];
  base_cMg[1][0]=n[1]; base_cMg[1][1]=o[1]; base_cMg[1][2]=a[1];
  base_cMg[2][0]=n[2]; base_cMg[2][1]=o[2]; base_cMg[2][2]=a[2];
  //ROS_INFO_STREAM("cMg is before rotate is...: " << std::endl << cMg);
  recalculate_cMg();

}

//Calculate cMg
void PCAutonomousGraspPlanning::recalculate_cMg(){

  intToConfig();
  if(aligned_grasp_){
    //aplicamos una rotación y traslación para posicionar la garra mejor
    vpHomogeneousMatrix grMgt0(0,along_,0,0,0,0);
    vpHomogeneousMatrix gMgrZ(0,0,0,0,0,1.57);
    vpHomogeneousMatrix gMgrX(0,0,0,1.57,0,0);		
    vpHomogeneousMatrix gMgrY(0,0,0,0,0,angle_);		
    vpHomogeneousMatrix grMgt(rad_,0,0,0,0,0);//-0.6
    vpHomogeneousMatrix cMgt;
    cMgt = grMgt0 * gMgrZ * gMgrX * gMgrY * grMgt;

    cMg = base_cMg * cMgt ;
    //ROS_INFO_STREAM("cMg1 is after rotate is...: " << std::endl << cMg);

  }else{
    //aplicamos una rotación y traslación para posicionar la garra mejor
    vpHomogeneousMatrix gMgr(0,0,0,0,angle_,0);
    vpHomogeneousMatrix grMgt0(rad_,0,0,0,0,0);
    vpHomogeneousMatrix grMgt1(0,along_,0,0,0,0);
    vpHomogeneousMatrix cMgt;
    cMgt = gMgr * grMgt0 * grMgt1;

    cMg = base_cMg  * cMgt ;// * test;
    //ROS_INFO_STREAM("cMg2 is after rotate is...: " << std::endl << cMg);
  }
  //Compute bMg and plan a grasp on bMg
  //vpHomogeneousMatrix bMg=bMc*cMg;
  //std::cerr << "bMg is: " << std::endl << bMg << std::endl;
}

/// Config from sliders to float values.
void PCAutonomousGraspPlanning::intToConfig(){
  bool old=aligned_grasp_;
  aligned_grasp_=ialigned_grasp==1?true:false;

  // @todo: Move defaults to other place.
  if(old!=aligned_grasp_){
    if(aligned_grasp_){iangle=45;irad=48;ialong=11;}
    else{ iangle=226;irad=50;ialong=0;}
  }

  angle_=iangle*(2.0*M_PI/360.0);
  rad_=-irad/100.0;
  along_=ialong/100.0;
}

///Ordenar en función de la proyección del punto sobre el eje definido 
///por axis_point_g y normal_g (globales)
bool PCAutonomousGraspPlanning::sortFunction(const PointT& d1, const PointT& d2)
{
  double t1 = (normal_g.x()*(d1.x-axis_point_g.x) + normal_g.y()*(d1.y-axis_point_g.y) + normal_g.z()*(d1.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));
  double t2 = (normal_g.x()*(d2.x-axis_point_g.x) + normal_g.y()*(d2.y-axis_point_g.y) + normal_g.z()*(d2.z-axis_point_g.z))/(pow(normal_g.x(),2) + pow(normal_g.y(),2) + pow(normal_g.z(),2));

  return t1 < t2;
}

///Obtiene los máximos y mínimos del cilindro para encontrar la altura del cilindro con un margen
///de descarte del 5%.
void PCAutonomousGraspPlanning::getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal)
{
  axis_point_g=axis_point; 
  normal_g=*normal;

  PointT max_p = axis_point;
  double max_t = 0.0;
  PointT min_p = axis_point;
  double min_t = 0.0;
  std::vector<PointT> list;
  PointT* k;

  //Al tener la lista de todos los puntos podemos descartar los que esten fuera de un 
  //determinado porcentaje (percentiles) para
  //Eliminar más outliers y ganar robustez.
  BOOST_FOREACH(const PointT& pt, cloud->points)
  {
    k=new PointT();
    k->x=pt.x*1;k->y=pt.y*1;k->z=pt.z*1;
    list.push_back(*k);
  }
  //Ordenamos con respecto al eje de direccion y tomamos P05 y P95
  std::sort(list.begin(), list.end(),  boost::bind(&PCAutonomousGraspPlanning::sortFunction, this, _1, _2));
  PointT max=list[(int)list.size()*0.05],min=list[(int)list.size()*0.95];
  //Proyección de los puntos reales a puntos sobre la normal.
  double t = (normal->x()*(max.x-axis_point.x) + normal->y()*(max.y-axis_point.y) + normal->z()*(max.z-axis_point.z))/(pow(normal->x(),2) + pow(normal->y(),2) + pow(normal->z(),2));
  PointT p;
  p.x = axis_point.x + normal->x() * t;
  p.y = axis_point.y + normal->y() * t;
  p.z = axis_point.z + normal->z() * t;
  *max_pt=p;
  t = (normal->x()*(min.x-axis_point.x) + normal->y()*(min.y-axis_point.y) + normal->z()*(min.z-axis_point.z))/(pow(normal->x(),2) + pow(normal->y(),2) + pow(normal->z(),2));
  p.x = axis_point.x + normal->x() * t;
  p.y = axis_point.y + normal->y() * t;
  p.z = axis_point.z + normal->z() * t;
  *min_pt=p;
}


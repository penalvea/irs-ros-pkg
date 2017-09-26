#include <laser_grasp/best_grasp.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "best_grasp");
  ros::NodeHandle nh;
  if(argc !=2){
    std::cerr<<"Syntax is: "<<argv[0]<<" main_scene.pcd\n"<<std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile(argv[1], *cloud)==-1){
    std::cout<<"Couldn't read file "<<std::endl;
  }

  BestGrasp bg;

  std::vector<pcl::PointXYZ> points=bg.getBestGrasp(cloud);






  Eigen::Vector3d y_axis(points[0].x-points[1].x,points[0].y-points[1].y, points[0].z-points[1].z );
  y_axis=y_axis.normalized();

  Eigen::Vector3d z_axis(points[4].x-points[5].x,points[4].y-points[5].y, points[4].z-points[5].z );
  z_axis=z_axis.normalized();

  Eigen::Vector3d x_axis=y_axis.cross(z_axis);
  x_axis=x_axis.normalized();



  vpHomogeneousMatrix bMg;
  bMg[0][0]=x_axis(0);  bMg[0][1]=y_axis(0);  bMg[0][2]=z_axis(0);  bMg[0][3]=points[4].x;
  bMg[1][0]=x_axis(1);  bMg[1][1]=y_axis(1);  bMg[1][2]=z_axis(1);  bMg[1][3]=points[4].y;
  bMg[2][0]=x_axis(2);  bMg[2][1]=y_axis(2);  bMg[2][2]=z_axis(2);  bMg[2][3]=points[4].z;
  bMg[3][0]=0;          bMg[3][1]=0;          bMg[3][2]=0;          bMg[3][3]=1;


  ARM5Arm arm;
  vpColVector dk(5);
  vpColVector res=arm.armIK(bMg);
  dk[0]=res[0];
  dk[1]=res[1];
  dk[2]=res[2];
  dk[3]=res[3];
  dk[4]=0;

  vpHomogeneousMatrix final_pose=arm.directKinematics(dk);

  std::cout<<bMg<<std::endl;
  std::cout<<"------------------"<<std::endl;
  std::cout<<dk<<std::endl;
  std::cout<<"*************************"<<std::endl;
  std::cout<<final_pose<<std::endl;
  std::cout<<"*************************"<<std::endl;


  pcl::PointXYZ x_point(points[4].x+x_axis(0), points[4].y+x_axis(1), points[4].z+x_axis(2));
  pcl::PointXYZ y_point(points[4].x+y_axis(0), points[4].y+y_axis(1), points[4].z+y_axis(2));
  pcl::PointXYZ z_point(points[4].x+z_axis(0), points[4].y+z_axis(1), points[4].z+z_axis(2));

  pcl::PointXYZ x_final(final_pose[0][3]+final_pose[0][0], final_pose[1][3]+final_pose[1][0], final_pose[2][3]+final_pose[2][0]);
  pcl::PointXYZ y_final(final_pose[0][3]+final_pose[0][1], final_pose[1][3]+final_pose[1][1], final_pose[2][3]+final_pose[2][1]);
  pcl::PointXYZ z_final(final_pose[0][3]+final_pose[0][2], final_pose[1][3]+final_pose[1][2], final_pose[2][3]+final_pose[2][2]);
  pcl::PointXYZ point_final(final_pose[0][3], final_pose[1][3], final_pose[2][3]);


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  //viewer->addLine(points[4], x_point, 1.0, 0.0, 0.0, "x");
  //viewer->addLine(points[4], y_point, 0.0, 1.0, 0.0, "y");
  //viewer->addLine(points[4], z_point, 0.0, 0.0, 1.0, "z");

  viewer->addLine(point_final, x_final, 1.0, 0.0, 0.0, "x_final");
  viewer->addLine(point_final, y_final, 0.0, 1.0, 0.0, "y_final");
  viewer->addLine(point_final, z_final, 0.0, 0.0, 1.0, "z_final");

  while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }






  Eigen::Vector3d y_axis_second(points[2].x-points[3].x,points[2].y-points[3].y, points[2].z-points[3].z );
  y_axis=y_axis.normalized();

  Eigen::Vector3d x_axis_second=y_axis_second.cross(z_axis);
  x_axis_second=x_axis_second.normalized();

  bMg[0][0]=x_axis_second(0);  bMg[0][1]=y_axis_second(0);  bMg[0][2]=z_axis(0);  bMg[0][3]=points[4].x;
  bMg[1][0]=x_axis_second(1);  bMg[1][1]=y_axis_second(1);  bMg[1][2]=z_axis(1);  bMg[1][3]=points[4].y;
  bMg[2][0]=x_axis_second(2);  bMg[2][1]=y_axis_second(2);  bMg[2][2]=z_axis(2);  bMg[2][3]=points[4].z;
  bMg[3][0]=0;          bMg[3][1]=0;          bMg[3][2]=0;          bMg[3][3]=1;

  res=arm.armIK(bMg);
  dk[0]=res[0];
  dk[1]=res[1];
  dk[2]=res[2];
  dk[3]=res[3];
  dk[4]=0;

final_pose=arm.directKinematics(dk);

std::cout<<bMg<<std::endl;
std::cout<<"------------------"<<std::endl;
std::cout<<dk<<std::endl;
std::cout<<"*************************"<<std::endl;
std::cout<<final_pose<<std::endl;
std::cout<<"*************************"<<std::endl;




pcl::PointXYZ x_final2(final_pose[0][3]+final_pose[0][0], final_pose[1][3]+final_pose[1][0], final_pose[2][3]+final_pose[2][0]);
pcl::PointXYZ y_final2(final_pose[0][3]+final_pose[0][1], final_pose[1][3]+final_pose[1][1], final_pose[2][3]+final_pose[2][1]);
pcl::PointXYZ z_final2(final_pose[0][3]+final_pose[0][2], final_pose[1][3]+final_pose[1][2], final_pose[2][3]+final_pose[2][2]);
pcl::PointXYZ point_final2(final_pose[0][3], final_pose[1][3], final_pose[2][3]);

pcl::PointXYZ x_point_second(points[4].x+x_axis_second(0), points[4].y+x_axis_second(1), points[4].z+x_axis_second(2));
pcl::PointXYZ y_point_second(points[4].x+y_axis_second(0), points[4].y+y_axis_second(1), points[4].z+y_axis_second(2));

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
viewer2->setBackgroundColor (0, 0, 0);
//viewer2->addCoordinateSystem (1.0);
viewer2->initCameraParameters ();
viewer2->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//viewer2->addLine(points[4], x_point_second, 1.0, 0.0, 0.0, "x");
//viewer2->addLine(points[4], y_point_second, 0.0, 1.0, 0.0, "y");
//viewer2->addLine(points[4], z_point, 0.0, 0.0, 1.0, "z");

viewer2->addLine(point_final2, x_final2, 1.0, 0.0, 0.0, "x_final");
viewer2->addLine(point_final2, y_final2, 0.0, 1.0, 0.0, "y_final");
viewer2->addLine(point_final2, z_final2, 0.0, 0.0, 1.0, "z_final");

while(!viewer2->wasStopped())
{
  viewer2->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}










  return 0;
}

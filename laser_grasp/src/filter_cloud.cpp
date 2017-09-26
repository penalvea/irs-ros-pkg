#include <laser_grasp/cloud_registration.h>

int main(int argc, char** argv){

  if(argc <2){
    std::cerr<<"Syntax is: "<<argv[0]<<" in_scene.pcd\n"<<std::endl;
    return (1);
  }
  for(int i=1; i< argc; i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[i], *cloud);
    CloudRegistration cr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered=cr.filterCloud(cloud);

    pcl::io::savePCDFile(argv[i], *cloud_filtered, false);
  }



  return 0;
}

#include <laser_grasp/cloud_registration.h>

int main(int argc, char** argv){

  if(argc <3){
    std::cerr<<"Syntax is: "<<argv[0]<<" main_scene.pcd scene1.1.pcd - scene2.2.pcd ...\n"<<std::endl;
    return (1);
  }

  std::vector<std::string> scenes1, scenes2;
  int i=2;
  while(std::strcmp(argv[i], "-")!=0){
    scenes1.push_back(argv[i]);
    std::cout<<argv[i]<<std::endl;
    i++;
  }
  std::cout<<"- salgo"<<std::endl;
  for(int j=i+1; j<argc; j++){
    std::cout<<j<<std::endl;
    scenes2.push_back(argv[j]);
    std::cout<<argv[j]<<std::endl;
  }

  std::cout<<scenes1.size()<<"  "<<scenes2.size()<<std::endl;
  CloudRegistration cr(argv[1], scenes1, scenes2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr complete=cr.registerClouds_onebyone(1);
  pcl::io::savePCDFile("./complete.pcd", *complete, false);



  return 0;
}

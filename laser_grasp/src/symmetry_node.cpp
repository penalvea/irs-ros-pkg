#include <laser_grasp/symmetry.h>


int main(int argc, char** argv){

  if(argc!=2){
    std::cerr<<"Syntax is: "<<argv[0]<<" main_scene.pcd\n"<<std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile(argv[1], *cloud)==-1){
    std::cout<<"Couldn't read file "<<std::endl;
  }

  Symmetry sym;
  sym.calculateSymmetry(cloud);

}

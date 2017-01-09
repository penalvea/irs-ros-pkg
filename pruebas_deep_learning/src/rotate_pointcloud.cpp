#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>



int main(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile("/home/toni/Escritorio/cube_2_2_10.pcd", *cloud);
  Eigen::Affine3f trans=Eigen::Affine3f::Identity();
  trans.rotate(Eigen::AngleAxisf(0.7, Eigen::Vector3f::UnitY()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*cloud, *transformed_cloud, trans);
  pcl::io::savePCDFileASCII("/home/toni/Escritorio/rotate.pcd", *transformed_cloud);


  std::vector< std::vector< std::vector< int > > > mat;
  for(int l=0; l<50; l++){
          std::vector<std::vector<int> > vec_vec;
          for(int m=0; m<50; m++){
                  std::vector<int> vec;
                  for(int n=0; n<50; n++){
                          vec.push_back(0);
                  }
                  vec_vec.push_back(vec);
          }
          mat.push_back(vec_vec);
  }

 for(int i=0; i< transformed_cloud->size(); i++){
    int x,y,z;

    x=transformed_cloud->points[i].x;
    y=transformed_cloud->points[i].y;
    z=transformed_cloud->points[i].z;
    std::cout<<x<<" "<<y<<" "<<z<<std::endl;
    mat[x][y][z]=1;
  }

 pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
 viewer->setBackgroundColor(0,0,0);
 viewer->addCube(0,1,0,1,0,1,0.5, 0.5, 0.5, "1");
 viewer->addCube(1,1.5,1,1.5,1,1.5,0.5, 0.5, 0.5, "2");
 viewer->addCoordinateSystem(1.0);
 viewer->initCameraParameters();
 while(!viewer->wasStopped()){
   viewer->spinOnce(100);
   boost::this_thread::sleep(boost::posix_time::microseconds(100000));
 }




 /* pcl::PointCloud<pcl::PointXYZ> side_cloud;
  side_cloud.width=1;
  int cont=0;
  for(int i=0; i<mat.size(); i++){
    for(int j=0; j<mat[0].size(); j++){
      bool found=false;
      for(int k=0; k<mat[0][0].size(); k++){
        if(mat[i][j][k]==1){
          if(!found){
            found=true;
            cont++;
            side_cloud.height=cont;
            side_cloud.resize(cont);
            side_cloud.points[cont-1].x=i;
            side_cloud.points[cont-1].y=j;
            side_cloud.points[cont-1].z=k;
          }
          else{
            mat[i][j][k]=0;
          }
        }
      }
    }
  }
  std::cout<<"salgo de todo"<<std::endl;
  pcl::io::savePCDFile("/home/toni/Escritorio/side.pcd", side_cloud);
std::cout<<"salgo de todo despues"<<std::endl;*/
	return 0;
}

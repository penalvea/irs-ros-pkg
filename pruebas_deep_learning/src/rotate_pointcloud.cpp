#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>



int main(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile("/home/toni/Escritorio/cube_39_8_32.pcd", *cloud);
  Eigen::Affine3f rot=Eigen::Affine3f::Identity();
  std::srand(std::time(0));
  float random=(std::rand()%1000)/1000.0*3.1415;
  random=2.62629;
  rot.rotate(Eigen::AngleAxisf(random, Eigen::Vector3f::UnitX()));
  random=(std::rand()%1000)/1000.0*3.1415;
  random=2.28387;
  rot.rotate(Eigen::AngleAxisf(random, Eigen::Vector3f::UnitY()));
  random=(std::rand()%1000)/1000.0*3.1415;
  random=0.109953;
  rot.rotate(Eigen::AngleAxisf(random, Eigen::Vector3f::UnitZ()));


  Eigen::Affine3f trans=Eigen::Affine3f::Identity();
  trans.translation() << -25, -25, -25;
  Eigen::Affine3f trans2=Eigen::Affine3f::Identity();
  trans2.translation() << 25, 25, 25;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*cloud, *transformed_cloud, trans);
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, rot);
  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, trans2);

  pcl::io::savePCDFileASCII("/home/toni/Escritorio/rotate.pcd", *transformed_cloud);


  std::vector< std::vector< std::vector< int > > > mat;
  std::vector< std::vector< std::vector< int > > > mat_translated;
  for(int l=0; l<50; l++){
    std::vector<std::vector<int> > vec_vec;
     std::vector<std::vector<int> > vec_vec_trans;
    for(int m=0; m<50; m++){
      std::vector<int> vec;
      std::vector<int> vec_trans;
      for(int n=0; n<50; n++){
        vec.push_back(0);
        vec_trans.push_back(0);
      }
      vec_vec.push_back(vec);
      vec_vec_trans.push_back(vec_trans);
    }
    mat.push_back(vec_vec);
    mat_translated.push_back(vec_vec_trans);
  }

  for(int i=0; i< transformed_cloud->size(); i++){
    int x,y,z;

    x=transformed_cloud->points[i].x;
    y=transformed_cloud->points[i].y;
    z=transformed_cloud->points[i].z;
    mat[x][y][z]=1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr side_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  side_cloud->width=1;
  int cont=0;
  for(int i=0; i<mat.size(); i++){
    for(int j=0; j<mat[0].size(); j++){
      bool found=false;
      for(int k=0; k<mat[0][0].size(); k++){
        if(mat[i][j][k]==1){
          if(!found){
            found=true;
            cont++;
            side_cloud->height=cont;
            side_cloud->resize(cont);
            side_cloud->points[cont-1].x=i;
            side_cloud->points[cont-1].y=j;
            side_cloud->points[cont-1].z=k;
          }
          else{
            mat[i][j][k]=0;
          }
        }
      }
    }
  }
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0,0,0);

  int max_x=0, max_y=0, max_z=0, min_x=50, min_y=50, min_z=50;
  for(int i=0; i<50; i++){
    for(int j=0; j<50; j++){
      for(int k=0; k<50; k++){
        if(mat[i][j][k]==1){
            if(i>max_x)
              max_x=i;
            if(i<min_x)
              min_x=i;
            if(j>max_y)
              max_y=j;
            if(j<min_y)
              min_y=j;
            if(k>max_z)
              max_z=k;
            if(k<min_z)
              min_z=k;
        }
      }
    }
  }
  int diff_x=((max_x+min_x)/2)-25;
  int diff_y=((max_y+min_y)/2)-25;
  int diff_z=((max_z+min_z)/2)-25;
  std::cout<<diff_x<<" "<<diff_y<<" "<<diff_z<<std::endl;
  cont=0;
  for(int i=0; i<50; i++){
    for(int j=0; j<50; j++){
      for(int k=0; k<50; k++){
        if(mat[i][j][k]==1){
            mat_translated[i+diff_x][j+diff_y][k+diff_z]=1;
            std::ostringstream name;
            name<<cont;
            cont++;
            viewer->addCube(i+diff_x, i+diff_x+1, j+diff_y, j+diff_y+1, k+diff_z, k+diff_z+1, 1.0, 0.0, 0.0, name.str() );
        }
      }
    }
  }

  viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "side cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "side cloud");
  viewer->addCoordinateSystem(50.0);
  viewer->initCameraParameters();
  while(!viewer->wasStopped()){
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  pcl::io::savePCDFile("/home/toni/Escritorio/side.pcd", *side_cloud);
	return 0;
}

/*
 * generate_models.cpp
 *
 *  Created on: 21/12/2016
 *      Author: toni
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>



int main() {

  /** CUBE **/
  for(int i=39; i<=39; i+=6){
    for(int j=8; j<=8; j+=6){
      for(int k=32; k<=32; k+=6){
        std::vector< std::vector< std::vector< int > > > cube;
        for(int l=0; l<50; l++){
          std::vector<std::vector<int> > vec_vec;
          for(int m=0; m<50; m++){
            std::vector<int> vec;
            for(int n=0; n<50; n++){
              vec.push_back(0);
            }
            vec_vec.push_back(vec);
          }
          cube.push_back(vec_vec);
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.height=1;
        cloud.is_dense=false;
        cloud.points.resize(cloud.width*cloud.height);

        int cont=0;
        for(float l=(int)(-i/2); l<(int)(i/2); l+=0.25){
          for(float m=(int)(-j/2); m<(int)(j/2); m+=0.25){
            for(float n=(int)(-k/2); n<(int)(k/2); n+=0.25){
              cont++;
              cloud.width=cont;
              cloud.points.resize(cont);
              int x=l, y=m, z=n;
              cube[25+x][25+y][25+z]=1;

              cloud.points[cont-1].x=25+l;
              cloud.points[cont-1].y=25+m;
              cloud.points[cont-1].z=25+n;


            }
          }
        }


        std::string name="/home/toni/Escritorio/cube_";
        std::ostringstream ss1;
        ss1<<i;
        name=name+ss1.str()+"_";
        std::ostringstream ss2;
        ss2<<j;
        name=name+ss2.str()+"_";
        std::ostringstream ss3;
        ss3<<k;
        name=name+ss3.str()+".pcd";
        pcl::io::savePCDFileASCII(name, cloud);

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0,0,0);
        int cont2=0;
        for(int i=0; i<cube.size(); i++){
          for(int j=0; j<cube[0].size(); j++){
            for(int k=0; k<cube[0][0].size(); k++){
              if(cube[i][j][k]==1){
                std::ostringstream name;
                name<<cont2;
                cont2++;
                viewer->addCube(i,i+1,j,j+1,k,k+1,0.5, 0.5, 0.5, name.str());
              }

            }
          }
        }
        viewer->addCoordinateSystem(50.0);
        viewer->initCameraParameters();
        while(!viewer->wasStopped()){
          viewer->spinOnce(100);
          boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
      }
    }
  }
  /** Cylinder **/

 /* for(int r=4; r<=20; r+=6){
    for(int h=4; h<=40; h+=6){
      std::vector< std::vector< std::vector< int > > > cylinder;
      for(int l=0; l<50; l++){
        std::vector<std::vector<int> > vec_vec;
        for(int m=0; m<50; m++){
          std::vector<int> vec;
          for(int n=0; n<50; n++){
            vec.push_back(0);
          }
          vec_vec.push_back(vec);
        }
        cylinder.push_back(vec_vec);
      }
      pcl::PointCloud<pcl::PointXYZ> cloud;
      int cont=1;
      cloud.width=1;
      std::cout<<"r="<<r<<"h="<<h<<std::endl;
      for(float height=-h/2; height<h/2; height+=0.25){
        for(float i=0; i<50; i+=0.25){
          for(float j=0; j<50; j+=0.25){
            if(std::sqrt(((i-25)*(i-25))+((j-25)*(j-25)))<=r){
              cylinder[i][j][25+height]=1;
              cloud.height=cont;
              cloud.points.resize(cont);
              cloud.points[cont-1].x=i;
              cloud.points[cont-1].y=j;
              cloud.points[cont-1].z=25+height;
              cont++;
            }

          }
        }
      }
      std::string name="/home/toni/Escritorio/cylinder_";
      std::ostringstream ss1;
      ss1<<r;
      name=name+ss1.str()+"_";
      std::ostringstream ss2;
      ss2<<h;
      name=name+ss2.str()+".pcd";
      pcl::io::savePCDFileASCII(name, cloud);
    }
  }*/

  /** cone **/

 /* for(int r=4; r<=20; r+=6){
          for(int h=4; h<=40; h+=6){
                  std::vector< std::vector< std::vector< int > > > cone;
                  for(int l=0; l<50; l++){
                          std::vector<std::vector<int> > vec_vec;
                          for(int m=0; m<50; m++){
                                  std::vector<int> vec;
                                  for(int n=0; n<50; n++){
                                          vec.push_back(0);
                                  }
                                  vec_vec.push_back(vec);
                          }
                          cone.push_back(vec_vec);
                  }
                  float angle=std::atan(float(r)/h);
                  pcl::PointCloud<pcl::PointXYZ> cloud;
                  int cont=1;
                  cloud.width=1;
                  for(float height=-h/2; height<h/2; height+=0.25){
                          float radius_height=std::tan(angle)*((h/2)+height);
                          for(float i=0; i<50; i+=0.25){
                                  for(float j=0; j<50; j+=0.25){
                                          if(std::sqrt(((i-25)*(i-25))+((j-25)*(j-25)))<=radius_height){
                                                  cone[i][j][25+height]=1;
                                                  cloud.height=cont;
                                                  cloud.points.resize(cont);
                                                  cloud.points[cont-1].x=i;
                                                  cloud.points[cont-1].y=j;
                                                  cloud.points[cont-1].z=25+height;
                                                  cont++;
                                          }

                                  }
                          }
                  }
                  std::string name="/home/toni/Escritorio/cone_";
                  std::ostringstream ss1;
                  ss1<<r;
                  name=name+ss1.str()+"_";
                  std::ostringstream ss2;
                  ss2<<h;
                  name=name+ss2.str()+".pcd";
                  pcl::io::savePCDFileASCII(name, cloud);
          }
  }*/

  /** sphere **/

  /*for (int r = 4; r <= 20; r += 6) {
    std::vector<std::vector<std::vector<int> > > cube;
    for (int l = 0; l < 50; l++) {
      std::vector<std::vector<int> > vec_vec;
      for (int m = 0; m < 50; m++) {
        std::vector<int> vec;
        for (int n = 0; n < 50; n++) {
          vec.push_back(0);
        }
        vec_vec.push_back(vec);
      }
      cube.push_back(vec_vec);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int cont = 1;
    cloud.width = 1;
    for (float i = 0; i < 50; i+=0.25) {
      for (float j = 0; j < 50; j+=0.25) {
        for (float k = 0; k < 50; k+=0.25) {
          if (std::sqrt(((i - 25) * (i - 25)) + ((j - 25) * (j - 25)) +
                        ((k - 25) * (k - 25))) <= r) {
            cube[i][j][k] = 1;
            cloud.height = cont;
            cloud.points.resize(cont);
            cloud.points[cont - 1].x = i;
            cloud.points[cont - 1].y = j;
            cloud.points[cont - 1].z = k;
            cont++;
          }
        }
      }
    }
    std::string name = "/home/toni/Escritorio/sphere_";
    std::ostringstream ss1;
    ss1 << r;
    name = name + ss1.str() + ".pcd";
    pcl::io::savePCDFileASCII(name, cloud);
  }*/
  return 0;
}

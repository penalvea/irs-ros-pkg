#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>


pcl::PointCloud<pcl::PointXYZ>::Ptr generateCube(int x, int y, int z){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->height=1;
  int cont=0;
  for(float i=(int)(-x/2); i<(int)(x/2); i+=0.5){
    for(float j=(int)(-y/2); j<(int)(y/2); j+=0.5){
      for(float k=(int)(-z/2); k<(int)(z/2); k+=0.5){
        cont++;
        cloud->width=cont;
        cloud->points.resize(cont);
        cloud->points[cont-1].x=50+i;
        cloud->points[cont-1].y=50+j;
        cloud->points[cont-1].z=50+k;
      }
    }
  }
  return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr generateCylinder(int radius, int height){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width=1;
  int cont=0;
  for(float h=-height/2; h<height/2; h+=0.5){
    for(float i=0; i<100; i+=0.5){
      for(float j=0; j<100; j+=0.5){
        if(std::sqrt(((i-50)*(i-50))+((j-50)*(j-50)))<=radius){
          cont++;
          cloud->height=cont;
          cloud->points.resize(cont);
          cloud->points[cont-1].x=i;
          cloud->points[cont-1].y=j;
          cloud->points[cont-1].z=50+h;
        }

      }
    }
  }
  return cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr generateCone(int radius, int height){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width=1;
  int cont=0;
  float angle=std::atan(float(radius)/height);
  for(float h=-height/2; h<height/2; h+=0.5){
    float radius_height=std::tan(angle)*((height/2)+h);
    for(float i=0; i<100; i+=0.5){
      for(float j=0; j<100; j+=0.5){
        if(std::sqrt(((i-50)*(i-50))+((j-50)*(j-50)))<=radius_height){
          cont++;
          cloud->height=cont;
          cloud->points.resize(cont);
          cloud->points[cont-1].x=i;
          cloud->points[cont-1].y=j;
          cloud->points[cont-1].z=50+h;

        }

      }
    }
  }
  return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr generateSphere(int radius){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width=1;
  int cont=0;
  for (float i = 0; i < 100; i+=0.5) {
    for (float j = 0; j < 100; j+=0.5) {
      for (float k = 0; k < 100; k+=0.5) {
        if (std::sqrt(((i - 50) * (i - 50)) + ((j - 50) * (j - 50)) + ((k - 50) * (k - 50))) <= radius) {
          cont++;
          cloud->height = cont;
          cloud->points.resize(cont);
          cloud->points[cont - 1].x = i;
          cloud->points[cont - 1].y = j;
          cloud->points[cont - 1].z = k;
        }
      }
    }
  }
  return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float rot_x, float rot_y, float rot_z){
  Eigen::Affine3f rot=Eigen::Affine3f::Identity();
  rot.rotate(Eigen::AngleAxisf(rot_x, Eigen::Vector3f::UnitX()));
  rot.rotate(Eigen::AngleAxisf(rot_y, Eigen::Vector3f::UnitY()));
  rot.rotate(Eigen::AngleAxisf(rot_z, Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f trans=Eigen::Affine3f::Identity();
  trans.translation() << -50, -50, -50;
  Eigen::Affine3f trans2=Eigen::Affine3f::Identity();
  trans2.translation() << 50, 50, 50;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*cloud, *rotated_cloud, trans);
  pcl::transformPointCloud(*rotated_cloud, *rotated_cloud, rot);
  pcl::transformPointCloud(*rotated_cloud, *rotated_cloud, trans2);
  return rotated_cloud;

}

std::vector< std::vector <std::vector< int > > > getMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  std::vector< std::vector< std::vector< int > > > mat;
  for(int l=0; l<100; l++){
    std::vector<std::vector<int> > vec_vec;
    for(int m=0; m<100; m++){
      std::vector<int> vec;
      for(int n=0; n<100; n++){
        vec.push_back(0);
      }
      vec_vec.push_back(vec);
    }
    mat.push_back(vec_vec);
  }
  for(int i=0; i<cloud->size(); i++){
    if(cloud->points[i].x>99 || cloud->points[i].x<0 || cloud->points[i].y>99 || cloud->points[i].y<0 || cloud->points[i].z>99 || cloud->points[i].z<0)
      std::cout<<"Error rotation, out of matrix: "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].x<<std::endl;
    else
      mat[cloud->points[i].x][cloud->points[i].y][cloud->points[i].z]=1;
  }
  return mat;
}

std::vector< std::vector <std::vector< int > > > getSideMatrix(std::vector< std::vector <std::vector< int > > > mat){
  std::vector< std::vector <std::vector< int > > > side_mat;
  for(int l=0; l<100; l++){
    std::vector<std::vector<int> > vec_vec;
    for(int m=0; m<100; m++){
      std::vector<int> vec;
      for(int n=0; n<100; n++){
        vec.push_back(0);
      }
      vec_vec.push_back(vec);
    }
    side_mat.push_back(vec_vec);
  }


  for(int i=0; i<mat.size(); i++){
    for(int j=0; j<mat[0].size(); j++){
      bool found=false;
      for(int k=0; k<mat[0][0].size(); k++){
        if(mat[i][j][k]==1){
          if(!found){
            side_mat[i][j][k]=1;
            found=true;
          }
        }
      }
    }
  }
  return side_mat;
}

std::vector<int> getDisplacement(std::vector< std::vector <std::vector< int > > > mat)
{
  int max_x=0, max_y=0, max_z=0, min_x=100, min_y=100, min_z=100;
  for(int i=0; i<mat.size(); i++){
    for(int j=0; j<mat[0].size(); j++){
      for(int k=0; k<mat[0][0].size(); k++){
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
  int disp_x=50-((max_x+min_x)/2);
  int disp_y=50-((max_y+min_y)/2);
  //int disp_z=((max_z+min_z)/2)-25;
  int disp_z=-min_z;
  std::vector<int> disp;
  disp.push_back(disp_x);
  disp.push_back(disp_y);
  disp.push_back(disp_z);
  return disp;
}

std::vector< std::vector <std::vector< int > > > moveMatrix(std::vector< std::vector <std::vector< int > > > mat, std::vector<int> disp){
  std::vector< std::vector <std::vector< int > > > moved_mat;
  for(int l=0; l<100; l++){
    std::vector<std::vector<int> > vec_vec;
    for(int m=0; m<100; m++){
      std::vector<int> vec;
      for(int n=0; n<100; n++){
        vec.push_back(0);
      }
      vec_vec.push_back(vec);
    }
    moved_mat.push_back(vec_vec);
  }
  for(int i=0; i<mat.size(); i++){
    for(int j=0; j<mat[0].size(); j++){
      for(int k=0; k<mat[0][0].size(); k++){
        if(mat[i][j][k]==1){
          if(i+disp[0]>99 || i+disp[0]<0 || j+disp[1]>99 || j+disp[1]<0 || k+disp[2]>99 || k+disp[2]<0)
            std::cout<<"Error move matrix, out of matrix: "<<i+disp[0]<<" "<<j+disp[1]<<" "<<k+disp[2]<<std::endl;
          else
            moved_mat[i+disp[0]][j+disp[1]][k+disp[2]]=1;
        }
      }
    }
  }
  return moved_mat;
}

void visualizeMat(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector< std::vector <std::vector< int > > > mat, float r, float g, float b, int start=0){
  int cont=0;
  for(int i=0; i<mat.size(); i++){
    for(int j=0; j<mat[0].size(); j++){
      for(int k=0; k<mat[0][0].size(); k++){
        if(mat[i][j][k]==1){
          std::ostringstream name;
          name<<cont+start;
          viewer->addCube(i, i+1, j, j+1, k, k+1, r, g, b, name.str());
          cont++;
        }
      }
    }
  }
}

void visualizePointCloud(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string name){
  viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
}

std::vector< std::vector< std::vector <std::vector< int > > > > generateMats(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float rand_x, float rand_y, float rand_z){
  std::vector< std::vector <std::vector< int > > > mat=getMatrix(rotatePointCloud(cloud, rand_x, rand_y, rand_z));
  std::vector< std::vector <std::vector< int > > > side_mat=getSideMatrix(mat);
  std::vector<int> displacement=getDisplacement(side_mat);
  std::vector< std::vector <std::vector< int > > > disp_side_mat=moveMatrix(side_mat, displacement);
  std::vector< std::vector <std::vector< int > > > disp_mat=moveMatrix(mat, displacement);

  std::vector< std::vector< std::vector <std::vector< int > > > > mats;
  mats.push_back(disp_mat);
  mats.push_back(disp_side_mat);
  return mats;
}

void writeMat(std::vector< std::vector <std::vector< int > > > mat, const std::string file_name){
  std::ofstream file(file_name.c_str());
  for(int i=0; i<mat.size(); i++){
    for(int j=0; j<mat[0].size(); j++){
      for(int k=0; k<mat[0][0].size(); k++){
        file<<mat[i][j][k]<<" ";
      }
    }
  }
  file.close();
}

int main(){
  int num_data=40000;
  int orientations=5;
  int cubes=3;
  int cylinders=2;
  int cones=2;
  int spheres=1;

  int cont=0;

  std::ofstream general("/home/toni/dataset/general.txt");
  general<<"id shape x/radius y/height z rot_x rot_y rot_z\n";
  std::string folder="/home/toni/dataset/first/";


  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0,0,0);
  viewer->addCoordinateSystem(100.0);
  viewer->initCameraParameters();


  std::srand(std::time(0));

  for(int i=0; i<num_data/(cubes+cylinders+cones+spheres)/orientations; i++){
    for (int cube=0; cube<cubes; cube++){
      int x=(std::rand()%45)+5;
      int y=(std::rand()%45)+5;
      int z=(std::rand()%45)+5;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cube_cloud=generateCube(x,y,z);
      for(int orientation=0; orientation<orientations; orientation++){
        float rand_x=(std::rand()%1000)/1000.0*3.1415;
        float rand_y=(std::rand()%1000)/1000.0*3.1415;
        float rand_z=(std::rand()%1000)/1000.0*3.1415;

        cont++;
        general<<cont<<" "<<"cube"<<" "<<x<<" "<<y<<" "<<z<<" "<<rand_x<<" "<<rand_y<<" "<<rand_z<<std::endl;
        std::vector< std::vector< std::vector <std::vector< int > > > > mats=generateMats(cube_cloud, rand_x, rand_y, rand_z);
        std::ostringstream id;
        id<<cont;
        writeMat(mats[0], folder+id.str()+"_complete.mat");
        writeMat(mats[1], folder+id.str()+"_side.mat");
        std::cout<<cont<<" cube"<<std::endl;
      }

    }

    for (int cylinder=0; cylinder<cylinders; cylinder++){
      int radius=(std::rand()%45)+5;
      int height=(std::rand()%45)+5;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cube_cloud=generateCylinder(radius,height);
      for(int orientation=0; orientation<orientations; orientation++){
        float rand_x=(std::rand()%1000)/1000.0*3.1415;
        float rand_y=(std::rand()%1000)/1000.0*3.1415;
        float rand_z=(std::rand()%1000)/1000.0*3.1415;
        cont++;
        general<<cont<<" "<<"cylinder"<<" "<<radius<<" "<<height<<" "<<"-1"<<" "<<rand_x<<" "<<rand_y<<" "<<rand_z<<std::endl;
        std::vector< std::vector< std::vector <std::vector< int > > > > mats=generateMats(cube_cloud, rand_x, rand_y, rand_z);
        std::ostringstream id;
        id<<cont;
        writeMat(mats[0], folder+id.str()+"_complete.mat");
        writeMat(mats[1], folder+id.str()+"_side.mat");
        std::cout<<cont<<" cylinder"<<std::endl;
      }

    }
    for (int cone=0; cone<cones; cone++){
      int radius=(std::rand()%45)+5;
      int height=(std::rand()%45)+5;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cube_cloud=generateCone(radius,height);
      for(int orientation=0; orientation<orientations; orientation++){
        float rand_x=(std::rand()%1000)/1000.0*3.1415;
        float rand_y=(std::rand()%1000)/1000.0*3.1415;
        float rand_z=(std::rand()%1000)/1000.0*3.1415;
        cont++;
        general<<cont<<" "<<"cone"<<" "<<radius<<" "<<height<<" "<<"-1"<<" "<<rand_x<<" "<<rand_y<<" "<<rand_z<<std::endl;
        std::vector< std::vector< std::vector <std::vector< int > > > > mats=generateMats(cube_cloud, rand_x, rand_y, rand_z);
        std::ostringstream id;
        id<<cont;
        writeMat(mats[0], folder+id.str()+"_complete.mat");
        writeMat(mats[1], folder+id.str()+"_side.mat");
        std::cout<<cont<<" cone"<<std::endl;
      }

    }
    for (int sphere=0; sphere<spheres; sphere++){
      int radius=(std::rand()%45)+5;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cube_cloud=generateSphere(radius);
      for(int orientation=0; orientation<orientations; orientation++){
        float rand_x=(std::rand()%1000)/1000.0*3.1415;
        float rand_y=(std::rand()%1000)/1000.0*3.1415;
        float rand_z=(std::rand()%1000)/1000.0*3.1415;
        cont++;
        general<<cont<<" "<<"sphere"<<" "<<radius<<" "<<"-1"<<" "<<"-1"<<" "<<rand_x<<" "<<rand_y<<" "<<rand_z<<std::endl;
        std::vector< std::vector< std::vector <std::vector< int > > > > mats=generateMats(cube_cloud, rand_x, rand_y, rand_z);
        std::ostringstream id;
        id<<cont;
        writeMat(mats[0], folder+id.str()+"_complete.mat");
        writeMat(mats[1], folder+id.str()+"_side.mat");
        std::cout<<cont<<" sphere"<<std::endl;
      }

    }



  }


  return 0;
}

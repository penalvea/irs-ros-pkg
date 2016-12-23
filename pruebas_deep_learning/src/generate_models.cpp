/*
 * generate_models.cpp
 *
 *  Created on: 21/12/2016
 *      Author: toni
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(){


	/** CUBE **/
	/*for(int i=4; i<=40; i+=6){
		for(int j=4; j<=40; j+=6){
			for(int k=4; k<=40; k+=6){
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
				cloud.width=i*j*k;
				cloud.height=1;
				cloud.is_dense=false;
				cloud.points.resize(cloud.width*cloud.height);
				int cont=0;
				for(int l=-i/2; l<i/2; l++){
					for(int m=-j/2; m<j/2; m++){
						for(int n=-k/2; n<k/2; n++){
							cube[25+l][25+m][25+n]=1;

							cloud.points[cont].x=25+l;
							cloud.points[cont].y=25+m;
							cloud.points[cont].z=25+n;
							cont++;
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

			}
		}
	}*/


	/** Cylinder **/

	/*for(int r=4; r<=20; r+=6){
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
			for(int height=-h/2; height<h/2; height++){
				for(int i=0; i<50; i++){
					for(int j=0; j<50; j++){
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

		for(int r=4; r<=20; r+=6){
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
				for(int height=-h/2; height<h/2; height++){
					float radius_height=std::tan(angle)*((h/2)+height);
					for(int i=0; i<50; i++){
						for(int j=0; j<50; j++){
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
		}
	return 0;
}




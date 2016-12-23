#include <laser_grasp/laser_grasp.h>
#include <iostream>
#include <pcl/features/moment_of_inertia_estimation.h>



typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

int m_x, m_y, m_event, m_flags;
bool m_is_new;

void LaserGrasp::force_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
	if(!force_init){
		force_[0]=msg->wrench.force.x;
		force_[1]=msg->wrench.force.y;
		force_[2]=msg->wrench.force.z;
		torque_[0]=msg->wrench.torque.x;
		torque_[1]=msg->wrench.torque.y;
		torque_[2]=msg->wrench.torque.z;
		force_init=true;
	}
	else{
		if(std::abs(force_[0]-msg->wrench.force.x)>150){
			force_filtered_[0]=msg->wrench.force.x-force_[0];
			force_[0]=msg->wrench.force.x;
		}
		if(std::abs(force_[1]-msg->wrench.force.y)>150){
			force_filtered_[1]=msg->wrench.force.y-force_[1];
			force_[1]=msg->wrench.force.y;
		}
		if(std::abs(force_[2]-msg->wrench.force.z)>150){
			force_filtered_[2]=msg->wrench.force.z-force_[2];
			force_[2]=msg->wrench.force.z;
		}
		if(std::abs(torque_[0]-msg->wrench.torque.x)>150){
			torque_filtered_[0]=msg->wrench.torque.x-torque_[0];
			torque_[0]=msg->wrench.torque.x;
		}
		if(std::abs(torque_[1]-msg->wrench.torque.y)>150){
			torque_filtered_[1]=msg->wrench.torque.y-torque_[1];
			torque_[1]=msg->wrench.torque.y;
		}
		if(std::abs(torque_[2]-msg->wrench.torque.z)>100){
			torque_filtered_[2]=msg->wrench.torque.z-torque_[2];
			torque_[2]=msg->wrench.torque.z;
		}
	}

}
void LaserGrasp::zero_force_sensor(){
	force_init=false;

	while(!force_init && ros::ok()){
		ros::spinOnce();
	}
	for(int i; i<3; i++){
		force_filtered_[i]=0;
		torque_filtered_[i]=0;
	}
	std::cout<<"torque despues de 0 "<<torque_filtered_[2]<<std::endl;
	sleep(2);
}


LaserGrasp::LaserGrasp(ros::NodeHandle& nh, std::string point_cloud_path, std::string image_topic, std::string info_topic, std::string arm_joint_state, std::string arm_command){
	nh_=nh;
	vg_.reset(new VirtualImage(nh_, image_topic, info_topic));
	point_cloud_pub_=nh.advertise<sensor_msgs::PointCloud2> ("/uwsim/point_cloud_loader",1);
	sensor_msgs::PointCloud2 msg;
	//pcl::toROSMsg(*cloud, msg);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile(point_cloud_path, *cloud)==-1){
		std::cout<<"Couldn't read file "<<std::endl;
	}
	std::cout<<cloud->width<<std::endl;
	cloud_=filter_cloud(cloud);
	std::cout<<cloud->width<<std::endl;



	cloud_indices_=segment_cloud(cloud_);
	std::cout<<"indices="<<cloud_indices_.size()<<std::endl;
	cloud_color_=color_cloud(cloud_);
	pcl::io::savePCDFile("/home/toni/JC_point_cloud.pcd", *cloud_color_, false);
	viewer_=rgbVis(cloud_color_);

	object_selected_=false;
	last_index_=-1;
	mouse_event_=new MouseEvent();
	mouse_event_->is_new=false;
	m_is_new=false;

	arm=new ARM5Arm(nh_, arm_joint_state, arm_command);
	force_sensor_sub_=nh_.subscribe<geometry_msgs::WrenchStamped>("/optoforce", 1, &LaserGrasp::force_sensor_callback, this);
	force_init=false;

}



pcl::PointCloud<pcl::PointXYZ>::Ptr LaserGrasp::filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.2, 1.8);
	pass.filter(*cloud_filtered);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_filtered);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered2);
	return cloud_filtered2;

}
std::vector<pcl::PointIndices> LaserGrasp::segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.01); // 1cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (250000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);


	return cluster_indices;

}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr LaserGrasp::color_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud, *cloud_color);
	for(int i=0; i< cloud_color->width; i++){
		cloud_color->points[i].r=255;
		cloud_color->points[i].g=255;
		cloud_color->points[i].b=255;
	}
	return cloud_color;
}


pcl::visualization::PCLVisualizer::Ptr LaserGrasp::rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "color cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "color cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
  }

void LaserGrasp::change_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
	viewer_->removePointCloud("color cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer_->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "color cloud");
}
void LaserGrasp::change_color(int index){

	if(last_index_!=-1){
		for(std::vector<int>::const_iterator pit = cloud_indices_[last_index_].indices.begin (); pit != cloud_indices_[last_index_].indices.end (); ++pit){
			cloud_color_->points[*pit].g=255;
			cloud_color_->points[*pit].b=255;
		}
	}
	for(std::vector<int>::const_iterator pit = cloud_indices_[index].indices.begin (); pit != cloud_indices_[index].indices.end (); ++pit){
		cloud_color_->points[*pit].g=0;
		cloud_color_->points[*pit].b=0;
	}
	last_index_=index;
	change_cloud(cloud_color_);
}



void LaserGrasp::mouse_callback_static(int event, int x, int y, int flags, void* param){
	LaserGrasp *self=static_cast<LaserGrasp*>(param);
	m_event=event;
	m_x=x;
	m_y=y;
	m_flags=flags;
	m_is_new=true;


}
float dist_line_point(cv::Point3f point, cv::Point3f line){
	cv::Point3f w=point-line;
	float c1=w.dot(line);
	float c2=line.dot(line);
	float b=c1/c2;
	cv::Point3f Pb=b*line;
	return cv::norm(point-Pb);
}
void LaserGrasp::mouse_callback(int event, int x, int y, int flags){
	if  ( event == cv::EVENT_LBUTTONDOWN || event==cv::EVENT_MOUSEMOVE){
		cv::Point3d point, line;
		line.x=(x-vg_->K.get_u0())/vg_->K.get_px();
		line.y=(y-vg_->K.get_v0())/vg_->K.get_py();
		line.z=1;
		float min_dist=-1;
		int index=-1;
		vpHomogeneousMatrix bMc, cMb;
		bMc.buildFrom(vpTranslationVector(-0.30347,0.045,0.08675), vpRotationMatrix(vpRxyzVector(0.0, 0.0, -1.57)));
		cMb=bMc.inverse();
		vpColVector bTp(4);
		for(int i=0; i<cloud_indices_.size(); i++){
			for(std::vector<int>::const_iterator pit = cloud_indices_[i].indices.begin (); pit != cloud_indices_[i].indices.end (); ++pit){
				/*point.y=cloud_color_->points[*pit].x;
				point.x=-cloud_color_->points[*pit].y;
				point.z=cloud_color_->points[*pit].z;*/
				bTp[0]=cloud_color_->points[*pit].x;
				bTp[1]=cloud_color_->points[*pit].y;
				bTp[2]=cloud_color_->points[*pit].z;
				bTp[3]=1;
				vpColVector cTp=cMb*bTp;
				point.x=cTp[0];
				point.y=cTp[1];
				point.z=cTp[2];
				float distance=dist_line_point(point, line);
				//std::cout<<i<<"    "<<distance<<std::endl;
				if(min_dist==-1){
					min_dist=distance;
					index=i;
				}
				else{
					if(distance<min_dist){
						min_dist=distance;
						index=i;
					}
				}

			}
		}
		if(index!=-1){
			if(index!=last_index_){
				change_color(index);
			}
			if(event==cv::EVENT_LBUTTONDOWN){
				object_selected_=true;
			}
		}
	}
}

void LaserGrasp::choose_object(){
	wait_camera();
	cv::namedWindow("Camera image", CV_WINDOW_NORMAL);
	cv::Mat img;
	vpImageConvert::convert(vg_->image, img);
	cv::setMouseCallback("Camera image", LaserGrasp::mouse_callback_static, NULL);
	cv::imshow("Camera image", img);
	while(ros::ok() && not object_selected_){
		cv::waitKey(100);
		viewer_->spinOnce(100);
		if(m_is_new){
			mouse_callback(m_event, m_x, m_y, m_flags);
			m_is_new=false;
		}
	}
	best_grasp();

	std::cout<<"Paso waitKey"<<std::endl;
}

void LaserGrasp::best_grasp(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_object_rgb->height=1;
	cloud_object_rgb->width=cloud_indices_[last_index_].indices.size();
	cloud_object_rgb->is_dense=true;
	for(std::vector<int>::const_iterator pit=cloud_indices_[last_index_].indices.begin(); pit!=cloud_indices_[last_index_].indices.end(); ++pit){
		cloud_object_rgb->points.push_back(cloud_color_->points[*pit]);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_object_rgb, *cloud_object);

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (cloud_object);
	feature_extractor.compute ();

	std::vector <float> moment_of_inertia;

	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia (moment_of_inertia);

	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter (mass_center);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	//viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->addPointCloud<pcl::PointXYZ> (cloud_object, "sample cloud");

	Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat (rotational_matrix_OBB);


	Eigen::Transform<float, 3, Eigen::Affine> bMpoint_eigen;
	Eigen::Translation<float, 3> bTmc(position);
	//Eigen::Vector3f displacement(0.0, 0.0, (-(max_point_OBB.z - min_point_OBB.z)/2)+0.1);
	bool bueno=false;
	float disp_z=-0.3, disp_x=0.0;
	vpColVector initial_joints;





	while(!bueno && disp_x>=-0.1){

		disp_z=-0.2;
		while(!bueno && disp_z<=0.0){
			Eigen::Vector3f displacement(disp_x, -0.05, disp_z);
			Eigen::Translation<float, 3> mcTpoint(displacement);
			//bMpoint_eigen=bTmc*quat*mcTpoint;
			bMpoint_eigen=bTmc*mcTpoint;


			/*pcl::PointXYZ point;
		point.x=bMpoint_eigen(0,3);
		point.y=bMpoint_eigen(1,3);
		point.z=bMpoint_eigen(2,3);
		viewer->addSphere(point, 0.01, "sphera");


		pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
		pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
		pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
		pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
		viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
		viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
		viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");*/



			vpHomogeneousMatrix bMpoint;
			/*for(int i=0; i<4; i++){
				for(int j=0; j<4; j++){
					std::cout<<bMpoint_eigen(i,j)<<" ";
				}
				std::cout<<std::endl;
			}*/

			bMpoint[0][3]=bMpoint_eigen(0,3);
			bMpoint[1][3]=bMpoint_eigen(1,3);
			bMpoint[2][3]=bMpoint_eigen(2,3);
			vpColVector joints(5), joints2(5);
			joints2=0;


			vpColVector maxJointLimits(4), minJointLimits(4);
			maxJointLimits[0]=30*M_PI/180;
			//maxJointLimits[0]=0.2;
			maxJointLimits[1]=90*M_PI/180;
			maxJointLimits[2]=145*M_PI/180;
			maxJointLimits[3]=360*M_PI/180;

			minJointLimits[0]=-90*M_PI/180;
			//minJointLimits[0]=0.2;
			minJointLimits[1]=0*M_PI/180;
			minJointLimits[2]=0*M_PI/180;
			minJointLimits[3]=-360*M_PI/180;

			joints=arm->armIK(bMpoint, maxJointLimits, minJointLimits);
			std::cout<<"Joints "<<std::endl;
			joints2[0]=joints[0];
			joints2[1]=joints[1];
			joints2[2]=joints[2];
			for(int i=0; i<joints.getRows(); i++){
				std::cout<<joints[i]<<" ";
			}
			std::cout<<std::endl;
			std::cout<<std::endl;
			std::cout<<std::endl;
			joints[3]=0.0;
			joints[4]=0.0;

			vpHomogeneousMatrix bMfinal=arm->directKinematics(joints2);
			/*for(int i=0; i<4; i++){
				for(int j=0; j<4; j++){

					std::cout<<bMfinal[i][j]<<" ";
				}
				std::cout<<std::endl;
			}*/
			std::cout<<"Pose differece"<<std::endl;
			std::cout<<bMpoint[0][3]<<"   "<<bMpoint[1][3]<<"  "<<bMpoint[2][3]<<" ---> "<<bMfinal[0][3]<<"   "<<bMfinal[1][3]<<"  "<<bMfinal[2][3]<<std::endl;
			std::cout<<"disp_x "<<disp_x<<" disp_z "<<disp_z<<std::endl;

			if((bMfinal.getCol(3)-bMpoint.getCol(3)).euclideanNorm()<0.01){
				std::cout<<"bueno"<<std::endl;
				bueno=true;
				initial_joints=joints2;
			}
			disp_z+=0.01;
		}
		disp_x-=0.01;
	}
	if(!bueno){
		return;
	}





	vpColVector joint_velocities(5);
	joint_velocities=0;
	joint_velocities[4]=0.3;
	while(arm->getCurrent()<1.0 && ros::ok()){
		arm->setJointVelocity(joint_velocities);
		ros::spinOnce();
		std::cout<<arm->getCurrent()<<std::endl;
	}
	std::cout<<"Hand opened"<<std::endl;

	vpColVector current_joints;
	arm->getJointValues(current_joints);
	initial_joints[3]=current_joints[3];
	initial_joints[4]=current_joints[4];
	while((initial_joints-current_joints).euclideanNorm()>0.017 && ros::ok()){
		std::cout<<"Error: "<<(initial_joints-current_joints).euclideanNorm()<<std::endl;
		arm->setJointVelocity((initial_joints-current_joints)*2.0);
		arm->getJointValues(current_joints);
		ros::spinOnce();
	}

	zero_force_sensor();

	vpColVector direction(6);
	direction=0;
	direction[0]=-0.1;
	direction[2]=0.1;
	bool floor_touched=false;
	while(!floor_touched && ros::ok()){

		std::cout<<"torque="<<torque_filtered_[2]<<std::endl;
		sleep(1);
		while(ros::ok() && std::abs(torque_filtered_[2])<10 ){
			arm->setCartesianVelocity(direction);
			ros::spinOnce();
			std::cout<<force_filtered_[0]<<"  "<<force_filtered_[1]<<"  "<<force_filtered_[2]<<"  "<<torque_filtered_[0]<<" "<<torque_filtered_[1]<<" "<<torque_filtered_[2]<<std::endl;
		}
		std::cout<<"Salgo porque he tocado"<<torque_filtered_[2]<<std::endl;
		sleep(1);
		vpHomogeneousMatrix bMe_current;
		arm->getPosition(bMe_current);
		if(bMe_current[2][3]>1.2){
			std::cout<<"piso tocado  "<<bMe_current[2][3]<<std::endl;
			sleep(1);
			floor_touched=true;
		}
		else{
			std::cout<<"objecto tocado  "<<bMe_current[2][3]<<std::endl;
			sleep(1);
			arm->getJointValues(current_joints);
			vpColVector rect_joints=current_joints;
			if(torque_filtered_>0){
				rect_joints[0]+=0.1;
			}
			else{
				rect_joints[0]-=0.1;
			}
			//rect_joints[2]+=0.05;
			while((rect_joints-current_joints).euclideanNorm()>0.015 && ros::ok()){
				std::cout<<"Error: "<<(rect_joints-current_joints).euclideanNorm()<<std::endl;
				arm->setJointVelocity((rect_joints-current_joints)*1.0);
				arm->getJointValues(current_joints);
				ros::spinOnce();
			}
			std::cout<<"me he desplazado "<<std::endl;
			sleep(1);
			zero_force_sensor();
			std::cout<<"pongo el senor a 0 y ahora esta: "<<torque_filtered_[2]<<std::endl;

		}
	}


	arm->getJointValues(current_joints);
	vpColVector no_touch_joints=current_joints;
	no_touch_joints[2]+=0.05;
	while((no_touch_joints-current_joints).euclideanNorm()>0.015 && ros::ok()){
		std::cout<<"Error: "<<(no_touch_joints-current_joints).euclideanNorm()<<std::endl;
		arm->setJointVelocity((no_touch_joints-current_joints)*1.0);
		arm->getJointValues(current_joints);
		ros::spinOnce();
	}

	joint_velocities[4]=-0.4;
	while(arm->getCurrent()<1.0 && ros::ok()){
		arm->setJointVelocity(joint_velocities);
		ros::spinOnce();
		std::cout<<arm->getCurrent()<<std::endl;
	}

	vpColVector final_position;
	arm->getJointValues(current_joints);
	final_position=current_joints;
	final_position[0]=0.0;
	final_position[1]=0.76;
	final_position[2]=1.48;

	while((final_position-current_joints).euclideanNorm()>0.015 && ros::ok()){
		std::cout<<"Error: "<<(final_position-current_joints).euclideanNorm()<<std::endl;
		arm->setJointVelocity((final_position-current_joints)*2.0);
		arm->getJointValues(current_joints);
		ros::spinOnce();
	}







	pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
	pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
	viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

	//Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	//Eigen::Quaternionf quat (rotational_matrix_OBB);
	viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");





	while(!viewer->wasStopped())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}





void LaserGrasp::wait_camera(){
	while(not vg_->ready() && ros::ok()){
		ros::spinOnce();
		usleep(1000);
	}
	std::cout<<"camera ready"<<std::endl;
}
void LaserGrasp::show_cloud(){
	int sep=255/cloud_indices_.size()*3;
	for(int i=0; i<cloud_indices_.size(); i++){
		for(std::vector<int>::const_iterator pit = cloud_indices_[i].indices.begin (); pit != cloud_indices_[i].indices.end (); ++pit){
			if(i%3==0){
				cloud_color_->points[*pit].r=255-(i*sep);
				cloud_color_->points[*pit].g=0+(i*sep);
				cloud_color_->points[*pit].b=0+(i*sep);
			}
			if(i%3==1){
				cloud_color_->points[*pit].r=0+(i*sep);
				cloud_color_->points[*pit].g=255-(i*sep);
				cloud_color_->points[*pit].b=0+(i*sep);
			}
			if(i%3==2){
				cloud_color_->points[*pit].r=0+(i*sep);
				cloud_color_->points[*pit].g=0+(i*sep);
				cloud_color_->points[*pit].b=255-(i*sep);
			}
		}
	}
	change_cloud(cloud_color_);
	while(ros::ok() && not viewer_->wasStopped()){
		viewer_->spinOnce(100);
		ros::spinOnce();
	}
}





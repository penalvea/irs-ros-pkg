#ifndef LASER_GRASP_H
#define LASER_GRASP_H

#include <ros/ros.h>
#include <mar_perception/VirtualImage.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpImage.h>
#include <visp/vpDiskGrabber.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpImagePoint.h>
#include <visp/vpPixelMeterConversion.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "opencv2/highgui/highgui.hpp"


class LaserGrasp{
	ros::NodeHandle nh_;
	VirtualImagePtr vg_;
	ros::Publisher point_cloud_pub_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_;
	std::vector<pcl::PointIndices> cloud_indices_;
	pcl::visualization::PCLVisualizer::Ptr viewer_;
	bool object_selected_;
	int last_index_;
	struct MouseEvent{
		int event;
		int x;
		int y;
		bool is_new;
	};
	MouseEvent *mouse_event_;
	ARM5Arm *arm;
	float force_[3], torque_[3], force_filtered_[3], torque_filtered_[3];
	bool force_init;
	ros::Subscriber force_sensor_sub_;
	void force_sensor_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	void zero_force_sensor();




public:
	LaserGrasp(ros::NodeHandle& nh, std::string point_cloud_path, std::string image_topic, std::string info_topic, std::string arm_joint_state, std::string arm_command);
	~LaserGrasp(){}
//	void execute();
	void choose_object();
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	std::vector<pcl::PointIndices> segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void change_color(int index);
	void change_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void mouse_callback(int event, int x, int y, int flags);
	static void mouse_callback_static(int event, int x, int y, int flags, void* param);
	void best_grasp();
	void wait_camera();
	void show_cloud();



};



#endif

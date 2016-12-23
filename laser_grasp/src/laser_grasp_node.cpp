#include <ros/ros.h>
#include <laser_grasp/laser_grasp.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "laser_grasp_node");
	ros::NodeHandle nh;
	LaserGrasp lg(nh, "/home/toni/3DReconstructions/stereo_left_real_good.pcd", "/stereo/left/image_rect", "/stereo/left/camera_info", "/arm5e/joint_state_angle", "/arm5e/command_angle");
	//LaserGrasp lg(nh, "/home/toni/3DReconstructions/stereo_left_real_good.pcd", "/uwsim/camera1", "/uwsim/camera1_info", "/arm5e/joint_state_angle", "/arm5e/command_angle");
	lg.choose_object();
	ros::spin();
	return 0;

}

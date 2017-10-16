#include <ros/ros.h>
#include <laser_grasp/laser_grasp.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "segment_cloud");
	ros::NodeHandle nh;
  LaserGrasp lg(nh, "/home/penalvea/Escritorio/MisArticulos/2017_OCEANS/point_clouds/filtered/anfora_broken/anfora_broken_eye.pcd", "/uwsim/rangecamera", "/uwsim/rangecamera_info", "/arm5e/joint_state_angle", "/arm5e/command_angle");

	lg.show_cloud();
	ros::spin();
	return 0;

}

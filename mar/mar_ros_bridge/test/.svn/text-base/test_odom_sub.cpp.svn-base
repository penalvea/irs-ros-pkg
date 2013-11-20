#include <mar_ros_bridge/Odometry.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_odom_sub");
	ros::NodeHandle nh;

	Odometry odom(nh, "/odom");

	ros::Rate r(5);
	while (ros::ok()) {
		ROS_INFO_THROTTLE(1.0, "Last message stamp: %d", odom.getMsg()->header.stamp.sec);

		ros::spinOnce();
		r.sleep();
	}
}

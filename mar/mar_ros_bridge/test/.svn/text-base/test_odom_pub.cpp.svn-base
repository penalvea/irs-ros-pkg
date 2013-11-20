#include <mar_ros_bridge/Odometry.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_odometry");
	ros::NodeHandle nh;

	Odometry odom;
	odom.setPublisher(nh, "/odom");

	ros::Rate r(4);
	while (ros::ok()) {
		odom.publish();

		ros::spinOnce();
		r.sleep();
	}
}

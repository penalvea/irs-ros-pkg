#include <mar_robot_arm5e/ARM5Arm.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "init_arm");
	ros::NodeHandle nh;
	ARM5Arm robot(nh);

	for (int i=0; i<10; i++) {
		ros::spinOnce();
		usleep(40000);
	}

	robot.init();

	vpColVector q(5);
	q=0;
	q[1]=0.2; q[2]=2.0;
	robot.setJointValues(q, true);

	return 0;
}

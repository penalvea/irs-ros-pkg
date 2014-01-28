#include <mar_robot_arm5e/ARM5Arm.h>
#include <visp/vpColVector.h>
#include <stdlib.h>

int main(int argc, char **argv) {
	if (argc != 6) {
                std::cerr << "Usage: " << argv[0] << " <q1> <q2> <q3> <q4> <q5>" << std::endl;
                exit(0);
        }

	ros::init(argc, argv, "setJointVelocity");
	ros::NodeHandle nh;
	ARM5Arm robot(nh);

	vpColVector qdot(5);
	for (int i=0; i<5; i++) qdot[i]=atof(argv[i+1]);

	ros::Rate rate(50);
	while (ros::ok()) {
		robot.setJointVelocity(qdot);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

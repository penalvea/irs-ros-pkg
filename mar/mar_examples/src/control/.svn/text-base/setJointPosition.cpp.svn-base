#include <mar_robot_arm5e/ARM5Arm.h>
#include <visp/vpColVector.h>
#include <iostream>
#include <stdlib.h>


int main(int argc, char **argv) {
	if (argc != 6) {
		std::cerr << "Usage: " << argv[0] << " <q0> <q1> <q2> <q3> <q4>" << std::endl;
		std::cerr << "Only q0, q1, q2, q3 implemented at the moment" << std::endl;
		exit(0);
	}

	ros::init(argc, argv, "setARM5JointPosition");
        ros::NodeHandle nh;
	ARM5Arm robot(nh);

	vpColVector q(5);
	for (int i=0; i<5; i++) q[i]=atof(argv[i+1]);

	robot.setJointValues(q, true);		

	ros::shutdown();	
	return 0;
}

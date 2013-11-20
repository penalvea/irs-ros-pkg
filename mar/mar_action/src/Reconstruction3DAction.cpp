#include <mar_action/Reconstruction3DAction.h>

/** Performs the logic necessary for 3D reconstruction using a laser mounted on an arm
 *  If not offline, the arm is initialized and moved to the starting configuration. It is then moved with joint velocities
 *  towards the final configuration, while image capture, peak detection and triangulation is performed at the same time.
 *  If offline (e.g running on a bagfile), no commands are send to the robot arm. If doing reconstruction from a mobile platform,
 *  an ESM tracker is also initialized, and used for estimating the motion of the mobile vehicle.
 *  Finishes on !ros::ok(), when a click is done in the image (if drawing enabled), or when the final arm configuration is reached.
 */
int Reconstruction3DAction::doAction() {
	if (!offline_ && robot_)  {
		//Initialize the arm zero offsets
		//robot_->init();
		//Move the arm to the initial position
		std::cout<<vp_scan_initial_posture_<<std::endl;
		robot_->setJointValues(vp_scan_initial_posture_, true);
	}
	//The main loop. Acquire image, track template, detect peaks, estimate motion, and reconstruction, while
	//controlling the arm through the scan:
	vpHomogeneousMatrix eMl;
	bool done=false;
	while (ros::ok() && !done) {
		ros::spinOnce();
		grabber_->acquire(*I_);

		//Update perceptions
		if (!fixed_base_ && tracker_ && mest_) {
			tracker_->perceive();
			mest_->perceive();
		}
		//std::cerr << "peak detector " << rec_->getPeakDetector() << std::endl;
		rec_->getPeakDetector()->perceive();
		rec_->perceive();

		draw();
		if (drawing_enabled && vpDisplay::getClick(*Idraw_, false)) done=true;

		//Make the scan motion: Send the arm to the final joint position with a suitable joint velocity
		if (!offline_) {
			double scale=1;
			vpColVector q;
			robot_->getJointValues(q);
			if ((vp_scan_final_posture_ - q).infinityNorm()>max_joint_velocity_) {
				//scale to the maximum allowed velocity
				scale=max_joint_velocity_/(vp_scan_final_posture_-q).infinityNorm();
			}
			if ((vp_scan_final_posture_ - q).infinityNorm() < position_tolerance_) break;
			robot_->setJointVelocity((vp_scan_final_posture_-q)*scale);
		}
	}
	//TODO: Park the arm

	//Save the point cloud in tmp
	//std::cerr << "Saving the point cloud in /tmp/pointcloud.pcd" << std::endl;
	//rec_->savePCD("/tmp/pointcloud.pcd");

	return SUCCESS;
}

void Reconstruction3DAction::draw() {
	if (drawing_enabled) {
		vpDisplay::display(*Idraw_);
		for (unsigned int i=0; i<rec_->getPeakDetector()->points.size(); i++) {
			vpDisplay::displayCross(*Idraw_,rec_->getPeakDetector()->points[i][0], rec_->getPeakDetector()->points[i][1], 5,vpColor::red);
			if (!fixed_base_ && tracker_) tracker_->draw(*Idraw_);
		}
		vpDisplay::flush(*Idraw_);
	}
}

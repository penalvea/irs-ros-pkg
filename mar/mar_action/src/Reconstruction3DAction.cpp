#include <mar_action/Reconstruction3DAction.h>
#include <boost/thread.hpp>


/** Performs the logic necessary for 3D reconstruction using a laser mounted on an arm
 *  If not offline, the arm is initialized and moved to the starting configuration. It is then moved with joint velocities
 *  towards the final configuration, while image capture, peak detection and triangulation is performed at the same time.
 *  If offline (e.g running on a bagfile), no commands are send to the robot arm. If doing reconstruction from a mobile platform,
 *  an ESM tracker is also initialized, and used for estimating the motion of the mobile vehicle.
 *  Finishes on !ros::ok(), when a click is done in the image (if drawing enabled), or when the final arm configuration is reached.
 */
int Reconstruction3DAction::doAction() {
	if (!offline_ && robot_)  {
		//Move the arm to the initial position
		std::cout<<vp_scan_initial_posture_<<std::endl;
		robot_->setJointValues(vp_scan_initial_posture_, true);
	}
	int cont=0;
	//The main loop. Acquire image, track template, detect peaks, estimate motion, and reconstruction, while
	//controlling the arm through the scan:
	vpHomogeneousMatrix eMl;
	done_=false;
	boost::thread threads[rec_.size()];
	for(int i=0; i<rec_.size(); i++){
		threads[i]=this->createThread(rec_[i]);
	}
	sleep(1);
	while (ros::ok() && !done_) {
		ros::spinOnce();

		//Update perceptions
		if (!fixed_base_ && tracker_ && mest_) {
			tracker_->perceive();
			mest_->perceive();
		}

		draw();

		/*for(int i=0; i<rec_.size(); i++){
			rec_[i]->perceive();

		}*/

		//if (drawing_enabled && vpDisplay::getClick(*Idraw_, false)) done=true;

		//Make the scan motion: Send the arm to the final joint position with a suitable joint velocity
		if (!offline_) {
			double scale=1;
			vpColVector q;
			robot_->getJointValues(q);
			if ((vp_scan_final_posture_ - q).infinityNorm()>max_joint_velocity_) {
				//scale to the maximum allowed velocity
				scale=max_joint_velocity_/(vp_scan_final_posture_-q).infinityNorm();
			}
			if ((vp_scan_final_posture_ - q).infinityNorm() < position_tolerance_) done_=true;
			robot_->setJointVelocity((vp_scan_final_posture_-q)*scale);
		}

		cont++;
	}
	std::cout<<"Cont="<<cont<<std::endl;
	for(int i=0; i<rec_.size(); i++){
		threads[i].join();
	}

	return SUCCESS;
}

void Reconstruction3DAction::draw() {
	if(drawing_enabled){
		for(int i=0; i<rec_.size(); i++){
			cv::Mat image;
			vpImage<vpRGBa> Ic;
			rec_[i]->getPeakDetector()->getGrabber()->acquire(Ic);
			vpImageConvert::convert(Ic, image);
			cv::namedWindow("camera"+boost::lexical_cast<std::string>(i), cv::WINDOW_AUTOSIZE);
			cv::line(image,cv::Point(0, rec_[i]->getPeakDetector()->getLimits()[0]),
				           cv::Point(Ic.getCols()-1, rec_[i]->getPeakDetector()->getLimits()[0]),
					       cv::Scalar(255,0,0));
			cv::line(image,cv::Point(0, rec_[i]->getPeakDetector()->getLimits()[1]),
							           cv::Point(Ic.getCols()-1, rec_[i]->getPeakDetector()->getLimits()[1]),
							           cv::Scalar(255,0,0));
			for(unsigned int j=0; j<rec_[i]->getPeakDetector()->points.size(); j++){

				//	std::cout<<rec_[i]->getPeakDetector()->points[i][1]<<"----"<<rec_[i]->getPeakDetector()->points[i][0]<<std::endl;
				cv::Vec3b & pixel=image.at<cv::Vec3b>(rec_[i]->getPeakDetector()->points[j][0], rec_[i]->getPeakDetector()->points[j][1]);
				pixel[0]=0;
				pixel[1]=0;
				pixel[2]=255;

			}
			cv::imshow("camera"+boost::lexical_cast<std::string>(i), image);
			cv::waitKey(5);
		}

	}

}

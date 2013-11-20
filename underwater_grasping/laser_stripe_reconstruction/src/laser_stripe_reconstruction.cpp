/*
 * Makes a laser scan and performs a 3D reconstruction
 *
 *  Created on: 19/06/2012
 *      Author: mprats
 */

#include <mar_perception/VirtualImage.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <mar_perception/ESMTracking.h>
#include <mar_perception/Reconstruction3D.h>
#include <mar_action/Reconstruction3DAction.h>
#include <mar_perception/MotionEstimator.h>
#include <mar_perception/PCGraspPlanning.h>
#include <mar_ros_bridge/mar_params.h>

#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>

/** Makes a 3D reconstruction of a surface with a laser stripe emitter mounted on an arm
 * Expects the following params to be set in the ROS parameter server:
 * image_topic: The topic where to listen for images
 * camera_info_topic: The topic with the camera info
 * offline: If 1, do not try to connect to the real robot
 * fixed: If 1, do not perform tracking nor motion estimation
 * output_basename: The full path to the output file names without extension, ie. /tmp/scan_2
 *
 * scan_initial_posture: a vector with the joint values where to start the scan (used if offline=0)
 * scan_final_posture: a vector with the joint values where to end the scan (used if offline=0)
 *
 * eMl: a six-element vector representing the laser pose wrt the end-effector. [x y z roll pitch yaw] format
 * bMc: a six-element vector representing the camera pose wrt the arm base. [x y z roll pitch yaw] format
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "arm5e_laser_reconstruction");
	ros::NodeHandle nh;

	std::string image_topic("");
	nh.getParam("image_topic", image_topic);
	std::string image_info_topic("");
	nh.getParam("camera_info_topic", image_info_topic);
	bool offline=true;
	bool fixed=false;
	nh.getParam("offline", offline);
	ROS_INFO("Offline: %d", offline);
	nh.getParam("fixed", fixed);
	ROS_INFO("Fixed: %d", fixed);

	std::string output_basename("output");
	nh.getParam("output_basename", output_basename);
	ROS_INFO_STREAM("output_basename: " << output_basename);
	vpColVector vp_scan_initial_posture=mar_params::paramToVispColVector(&nh, "scan_initial_posture");
	std::cout<<"initial posture "<<vp_scan_initial_posture[0]<<std::endl;

	vpColVector vp_scan_final_posture=mar_params::paramToVispColVector(&nh, "scan_final_posture");

	ARM5ArmPtr robot(new ARM5Arm(nh, "g500/joint_state", "g500/joint_state_command"));

	VirtualImagePtr grabber(new VirtualImage(nh,image_topic,image_info_topic));

	//Wait for images
	while (ros::ok() && !grabber->ready()) {
		ros::spinOnce();
	}

	boost::shared_ptr<vpImage<vpRGBa> > I(new vpImage<vpRGBa>()), Iorigin(new vpImage<vpRGBa>());
	grabber->open(*I);
	grabber->acquire(*I);
	std::cerr << "Image is " << I->getRows() << "x" << I->getCols() << std::endl;
	vpDisplayX window(*I);
	vpDisplay::display(*I);
	vpDisplay::flush(*I);
	*Iorigin=*I;

	//Let the user indicate the template and start tracking:
	ESMTrackingPtr tracker;
	ESMMotionEstimatorPtr mest;
	if (!fixed) {
		tracker=ESMTrackingPtr(new ESMTracking(I.get()));
        	tracker->perceive();
		mest=ESMMotionEstimatorPtr(new ESMMotionEstimator(tracker, grabber->K));
		mest->setZ(1.0); //Depth at the UJI water tank. FIXME: should get it from somewhere
	}

	MiquelSubPixelLaserPeakDetectorPtr peak_detector(new MiquelSubPixelLaserPeakDetector(grabber));
	//SimpleSubPixelLaserPeakDetectorPtr peak_detector(new SimpleSubPixelLaserPeakDetector(grabber));
	ArmLaserReconstruction3DPtr rec(new ArmLaserReconstruction3D(peak_detector, robot, tracker, mest, grabber));
	vpHomogeneousMatrix eMl=mar_params::paramToVispHomogeneousMatrix(&nh, "eMl");
    rec->setLaserToEef(eMl);

	//Camera to arm base calibration
	//vpHomogeneousMatrix bMc=mar_params::paramToVispHomogeneousMatrix(&nh, "bMc");
	vpHomogeneousMatrix bMc(-0.18, 0.0, -0.01, 0, 0, 0), auxRotZ(0,0,0,0,0,-1.57), auxRotX(0,0,0,0.2,0,0);
	bMc=bMc*auxRotZ*auxRotX;
	rec->setCameraToBase(bMc);
	
	Reconstruction3DAction action(I, grabber, robot, tracker, rec, mest);
	action.enableDrawing(boost::shared_ptr<vpImage<vpRGBa> >(I));
	action.setOffline(offline);
	action.setFixedBase(fixed);

	action.setInitialPosture(vp_scan_initial_posture);
	action.setFinalPosture(vp_scan_final_posture);
	action.doAction();
	//save pcd
	rec->savePCD(output_basename+std::string(".pcd"));
	std::cout << "Point cloud saved into " << output_basename+std::string(".pcd") << std::endl;
	//save camera view at origin
	vpImageIo::write(*Iorigin, output_basename+std::string(".ppm"));
	std::cout << "Reference image saved into " << output_basename+std::string(".ppm") << std::endl;
	//save camera parameters
	vpMatrix::saveMatrix(output_basename+std::string(".cam"), grabber->K.get_K());

	return 1;
}

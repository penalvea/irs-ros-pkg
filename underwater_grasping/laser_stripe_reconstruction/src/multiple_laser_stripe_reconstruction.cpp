/*
 * Makes a laser scan and performs a 3D reconstruction using an eye in hand camera
 *
 *  Created on: 17/07/2014
 *      Author: penalvea
 */

#include <mar_perception/VirtualImage.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <mar_perception/Reconstruction3D.h>
#include <mar_perception/ArmLaserReconstruction3DEye.h>
#include <mar_perception/ArmLaserReconstruction3DStatic.h>
#include <mar_action/Reconstruction3DAction.h>
#include <mar_perception/PCGraspPlanning.h>
#include <mar_ros_bridge/mar_params.h>

#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>



/** Makes a 3D reconstruction of a surface with a laser stripe emitter mounted on an arm and using a camera mounted on the same arm
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "multiple_arm5e_laser_reconstruction");
	ros::NodeHandle nh;

	bool offline=true;
	bool fixed=true;
	nh.getParam("offline", offline);
	ROS_INFO("Offline: %d", offline);
	nh.getParam("fixed", fixed);
	ROS_INFO("Fixed: %d", fixed);


	std::string joint_state_topic("uwsim/joint_state");
	nh.getParam("joint_state_topic", joint_state_topic);
	std::string joint_state_command_topic("uwsim/joint_state_command");
	nh.getParam("joint_state_command_topic",  joint_state_command_topic);
	ARM5ArmPtr robot(new ARM5Arm(nh, joint_state_topic, joint_state_command_topic));
	vpColVector vp_scan_initial_posture=mar_params::paramToVispColVector(&nh, "scan_initial_posture");
	vpColVector vp_scan_final_posture=mar_params::paramToVispColVector(&nh, "scan_final_posture");

	std::vector<Reconstruction3DPtr> rec_vector;
	std::vector<std::string> base_name_vector;
	XmlRpc::XmlRpcValue cameras;
	nh.getParam("cameras", cameras);
	for(int i=0; i<cameras.size(); i++){
		bool correct_parameters=true;
		VirtualImagePtr grabber;
		LaserPeakDetectorPtr detector;
		std::string camera;
		camera=static_cast<std::string>(cameras[i]);
		std::string image_topic, camera_info_topic;
		if(!nh.getParam(camera+"/image", image_topic) || !nh.getParam(camera+"/camera_info", camera_info_topic)){
			std::cerr<<"Error reading the "<<camera<<" camera topics"<<std::endl;
			correct_parameters=false;
		}
		else{
			grabber.reset(new VirtualImage(nh, image_topic, camera_info_topic));
			while(ros::ok() && !grabber->ready()){
				ros::spinOnce();
			}
		}
		if(correct_parameters){
			std::string peak_detector_type;
			if(nh.getParam(camera+"/peak_detector_type", peak_detector_type)){
				if(peak_detector_type=="simple"){
					detector.reset(new SimpleLaserPeakDetector(grabber));
				}
				else if(peak_detector_type=="simple_subpixel"){
					detector.reset(new SimpleSubPixelLaserPeakDetector(grabber));
				}
				else if(peak_detector_type=="last_image"){
					detector.reset(new LastImageSubPixelLaserPeakDetector(grabber));
				}
				else if(peak_detector_type=="hsv"){
					detector.reset(new HSVSubPixelLaserPeakDetector(grabber));
				}
				else if(peak_detector_type=="simulation"){
					detector.reset(new SimulationLaserPeakDetector(grabber));
				}
				else{
					correct_parameters=false;
					std::cerr<<"Invalid type of detector for the"<<camera<<" camera."<<std::endl;
				}
			}
			else{
				correct_parameters=false;
				std::cerr<<"Error reading the peak detector type for the "<<camera<<" camera."<<std::endl;
			}
		}
		if(correct_parameters){
			std::string reconstruction_type;
			if(nh.getParam(camera+"/reconstruction_type", reconstruction_type)){
				if(reconstruction_type=="static"){
					vpHomogeneousMatrix bMc=mar_params::paramToVispHomogeneousMatrix(&nh, camera+"/bMc");
					vpHomogeneousMatrix eMl=mar_params::paramToVispHomogeneousMatrix(&nh, camera+"/eMl");
					ArmLaserReconstruction3DStaticPtr rec(new ArmLaserReconstruction3DStatic(detector, robot));
					rec->setCameraToBase(bMc);
					rec->setLaserToEef(eMl);
					double max_radius, min_radius;
					if(nh.getParam(camera+"/min_radius_WS", min_radius) && nh.getParam(camera+"/max_radius_WS", max_radius)){
						rec->getPeakDetector()->setRadiusMin(min_radius);
						rec->getPeakDetector()->setRadiusMax(max_radius);
					}
					rec_vector.push_back(rec);
				}
				else if(reconstruction_type=="eye"){
					std::cout<<"aqui3"<<std::endl;
					vpHomogeneousMatrix eMc=mar_params::paramToVispHomogeneousMatrix(&nh, camera+"/eMc");
					vpHomogeneousMatrix eMl=mar_params::paramToVispHomogeneousMatrix(&nh, camera+"/eMl");
					ArmLaserReconstruction3DEyePtr rec(new ArmLaserReconstruction3DEye(detector, robot));
					rec->setCameraToEef(eMc);
					rec->setLaserToEef(eMl);
					double max_radius, min_radius;
					if(nh.getParam(camera+"/min_radius_WS", min_radius) && nh.getParam(camera+"/max_radius_WS", max_radius)){
						rec->getPeakDetector()->setRadiusMin(min_radius);
						rec->getPeakDetector()->setRadiusMax(max_radius);
					}
					rec_vector.push_back(rec);
				}
				else{
					correct_parameters=false;
					std::cerr<<"Invalid type of 3D reconstructor for the"<<camera<<" camera."<<std::endl;
				}
			}
			else{
				correct_parameters=false;
				std::cerr<<"Error reading the 3D reconstructor type for the "<<camera<<" camera."<<std::endl;
			}
		}
		if(correct_parameters){
			std::string base_name;
			if(nh.getParam(camera+"/base_name", base_name)){
				base_name_vector.push_back(base_name);
			}
			else{
				correct_parameters=false;
				rec_vector.pop_back();
				std::cerr<<"Error reading the base name for the "<<camera<<" camera."<<std::endl;
			}
		}
		if(correct_parameters){
			std::cout<<camera<<" camera loaded without errors."<<std::endl;
		}
	}

	if(rec_vector.size()==0){
		std::cerr<<"No camera loaded."<<std::endl;
	}
	else{
		Reconstruction3DAction action(robot, rec_vector);

		action.enableDrawing();
		action.setOffline(offline);
		action.setFixedBase(fixed);

		action.setInitialPosture(vp_scan_initial_posture);
		action.setFinalPosture(vp_scan_final_posture);
		action.doAction();

		for(int i=0; i<rec_vector.size(); i++){
			rec_vector[i]->savePCD(base_name_vector[i]+".pcd");
			vpMatrix::saveMatrix(base_name_vector[i]+".cam", rec_vector[i]->getPeakDetector()->getGrabber()->K.get_K());
		}
	}

	return 1;
}

/*
 * Makes a laser scan and performs a 3D reconstruction using an eye in hand camera
 *
 *  Created on: 17/07/2014
 *      Author: Toni Peñalver
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

#include <std_srvs/Empty.h>

/** Makes a 3D reconstruction of a surface with a laser stripe emitter mounted on an arm and using a camera mounted on the same arm
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multiple_arm5e_laser_reconstruction");
  ros::NodeHandle nh;

  bool offline = true;
  bool fixed = true;
  bool draw = false;
  bool publish_point_cloud = false;
  nh.getParam("offline", offline);
  ROS_INFO("Offline: %d", offline);
  nh.getParam("fixed", fixed);
  ROS_INFO("Fixed: %d", fixed);
  nh.getParam("draw", draw);
  ROS_INFO("Drawing: %d", draw);
  nh.getParam("publish_point_cloud", publish_point_cloud);
  ROS_INFO("Publishing point cloud: %d", publish_point_cloud);

  std::string joint_state_topic("uwsim/joint_state");
  nh.getParam("joint_state_topic", joint_state_topic);
  std::string joint_state_command_topic("uwsim/joint_state_command");
  nh.getParam("joint_state_command_topic", joint_state_command_topic);
  ARM5ArmPtr robot(new ARM5Arm(nh, joint_state_topic, joint_state_command_topic));
  vpColVector vp_scan_initial_posture = mar_params::paramToVispColVector(&nh, "scan_initial_posture");
  vpColVector vp_scan_final_posture = mar_params::paramToVispColVector(&nh, "scan_final_posture");

  std::vector<Reconstruction3DPtr> rec_vector;
  std::vector<std::string> base_name_vector;
  XmlRpc::XmlRpcValue cameras;
  std::vector<vpImage<vpRGBa> > initial_images;
  nh.getParam("cameras", cameras);
  for (int i = 0; i < cameras.size(); i++)
  {
    bool correct_parameters = true;
    VirtualImagePtr grabber;
    LaserPeakDetectorPtr detector;
    std::string camera;
    camera = static_cast<std::string>(cameras[i]);
    std::string image_topic, camera_info_topic;
    if (!nh.getParam(camera + "/image", image_topic) || !nh.getParam(camera + "/camera_info", camera_info_topic))
    {
      std::cerr << "Error reading the " << camera << " camera topics" << std::endl;
      correct_parameters = false;
    }
    else
    {
      grabber.reset(new VirtualImage(nh, image_topic, camera_info_topic));
      while (ros::ok() && !grabber->ready())
      {
        ros::spinOnce();
      }
      initial_images.push_back(grabber->image);
    }
    if (correct_parameters)
    {
      std::string peak_detector_type;
      if (nh.getParam(camera + "/peak_detector_type", peak_detector_type))
      {
        if (peak_detector_type == "simple")
        {
          detector.reset(new SimpleLaserPeakDetector(grabber));
        }
        else if (peak_detector_type == "simple_subpixel")
        {
          detector.reset(new SimpleSubPixelLaserPeakDetector(grabber));
        }
        else if (peak_detector_type == "last_image")
        {
          detector.reset(new LastImageSubPixelLaserPeakDetector(grabber));
        }
        else if (peak_detector_type == "hsv")
        {
          detector.reset(new HSVSubPixelLaserPeakDetector(grabber));
        }
        else if (peak_detector_type == "simulation")
        {
          detector.reset(new SimulationLaserPeakDetector(grabber));
        }
        else
        {
          correct_parameters = false;
          std::cerr << "Invalid type of detector for the" << camera << " camera." << std::endl;
        }
      }
      else
      {
        correct_parameters = false;
        std::cerr << "Error reading the peak detector type for the " << camera << " camera." << std::endl;
      }
    }
    if (correct_parameters)
    {
      std::string reconstruction_type;
      if (nh.getParam(camera + "/reconstruction_type", reconstruction_type))
      {
        if (reconstruction_type == "static")
        {
          vpHomogeneousMatrix bMc = mar_params::paramToVispHomogeneousMatrix(&nh, camera + "/bMc");
          vpHomogeneousMatrix eMl = mar_params::paramToVispHomogeneousMatrix(&nh, camera + "/eMl");
          ArmLaserReconstruction3DStaticPtr rec(new ArmLaserReconstruction3DStatic(detector, robot));
          rec->setCameraToBase(bMc);
          rec->setLaserToEef(eMl);
          double max_radius, min_radius;
          if (nh.getParam(camera + "/min_radius_WS", min_radius) && nh.getParam(camera + "/max_radius_WS", max_radius))
          {
            rec->getPeakDetector()->setRadiusMin(min_radius);
            rec->getPeakDetector()->setRadiusMax(max_radius);
          }
          rec_vector.push_back(rec);
        }
        else if (reconstruction_type == "eye")
        {
          vpHomogeneousMatrix eMc = mar_params::paramToVispHomogeneousMatrix(&nh, camera + "/eMc");
          vpHomogeneousMatrix eMl = mar_params::paramToVispHomogeneousMatrix(&nh, camera + "/eMl");
          ArmLaserReconstruction3DEyePtr rec(new ArmLaserReconstruction3DEye(detector, robot));
          rec->setCameraToEef(eMc);
          rec->setLaserToEef(eMl);
          double max_radius, min_radius;
          if (nh.getParam(camera + "/min_radius_WS", min_radius) && nh.getParam(camera + "/max_radius_WS", max_radius))
          {
            rec->getPeakDetector()->setRadiusMin(min_radius);
            rec->getPeakDetector()->setRadiusMax(max_radius);
          }
          std::cout << "min_radius=" << min_radius << std::endl;
          std::cout << "max_radius=" << max_radius << std::endl;
          rec_vector.push_back(rec);
        }
        else
        {
          correct_parameters = false;
          std::cerr << "Invalid type of 3D reconstructor for the" << camera << " camera." << std::endl;
        }
      }
      else
      {
        correct_parameters = false;
        std::cerr << "Error reading the 3D reconstructor type for the " << camera << " camera." << std::endl;
      }
    }
    if (correct_parameters)
    {
      std::string base_name;
      if (nh.getParam(camera + "/base_name", base_name))
      {
        base_name_vector.push_back(base_name);
      }
      else
      {
        correct_parameters = false;
        rec_vector.pop_back();
        std::cerr << "Error reading the base name for the " << camera << " camera." << std::endl;
      }
    }
    if (correct_parameters)
    {
      std::cout << camera << " camera loaded without errors." << std::endl;
    }
  }

  if (rec_vector.size() == 0)
  {
    std::cerr << "No camera loaded." << std::endl;
  }
  else
  {
    Reconstruction3DActionPtr action(new Reconstruction3DAction(robot, rec_vector));

    if (draw)
    {
      action->enableDrawing();
    }
    else
    {
      action->disableDrawing();
    }
    action->setOffline(offline);
    action->setFixedBase(fixed);
    action->setPointCloudPublished(publish_point_cloud);

    //for(double j=-1.5; j<=1.5; j+=0.5){
    //std::cout<<"j="<<j<<std::endl;
    //vp_scan_initial_posture[0]=j;
    //vp_scan_final_posture[0]=j;
    action->setInitialPosture(vp_scan_initial_posture);
    action->setFinalPosture(vp_scan_final_posture);

    action->doAction();
    for (int i = 0; i < rec_vector.size(); i++)
    {
      rec_vector[i]->savePCD(base_name_vector[i] + ".pcd");
      vpMatrix::saveMatrix(base_name_vector[i] + ".cam", rec_vector[i]->getPeakDetector()->getGrabber()->K.get_K());
      vpImageIo::write(initial_images[i], base_name_vector[i]+".ppm");
    }
    /*
     double time=0;
     rec_vector[0]->getPeakDetector()->setRadiusMax(-1);
     rec_vector[0]->getPeakDetector()->setRadiusMin(-1);

     for(int j=0; j<1; j++){
     action->doAction();
     time+=rec_vector[0]->getDuration().toSec()/rec_vector[0]->getIterations();
     std::cout<<time/(j+1)<<std::endl;
     }
     std::cout<<"-1 -- -1"<<"   "<<time<<std::endl;

     double max=1.5;
     for(double min=0.1; min<=0.8; min+=0.2){
     for(int i=0; i<rec_vector.size(); i++){
     rec_vector[i]->getPeakDetector()->setRadiusMax(max);
     rec_vector[i]->getPeakDetector()->setRadiusMin(min);
     }
     time=0;
     for(int j=0; j<1; j++){

     //std_srvs::Empty::Request request;
     //std_srvs::Empty::Response response;
     //ros::service::call("/StartBench", request, response);
     action->doAction();
     time+=rec_vector[0]->getDuration().toSec()/rec_vector[0]->getIterations();
     for(int i=0; i<rec_vector.size(); i++){
     std::stringstream ss;
     ss<<j;
     rec_vector[i]->savePCD(base_name_vector[i]+ss.str()+".pcd");
     vpMatrix::saveMatrix(base_name_vector[i]+ss.str()+".cam", rec_vector[i]->getPeakDetector()->getGrabber()->K.get_K());
     rec_vector[i]->reset();
     }
     }
     std::cout<<min<<" -- "<<max<<"   "<<time<<std::endl;
     max-=0.2;
     }*/
  }

  return 0;
}

#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <mar_perception/ESMTracking.h>
#include <mar_perception/VirtualImage.h>

int main(int argc, char **argv){

  if (argc!=3) {
  	std::cerr << "Usage: " << argv[0] << " <image_topic> <image_info_topic>" << std::endl;
	exit(0);
  }

  std::string image_topic(argv[1]);
  std::string image_info_topic(argv[2]);

  ros::init(argc, argv, "esm_tracking");
  ros::NodeHandle nh;
 
  vpImage<vpRGBa> Ic; // Color image

  // Declare a framegrabber able to read ROS images
  VirtualImage g(nh,image_topic,image_info_topic);
  while (!g.ready() && ros::ok()) {ros::spinOnce();}

  // Open the framegrabber by loading the first image of the sequence
  g.open(Ic) ;
  g.acquire(Ic);
  vpDisplayX window(Ic);
  vpDisplay::display(Ic);
  vpDisplay::flush(Ic);
  
  ESMTracking esm(&Ic);
  while(ros::ok())
  {
    //std::cerr << "ros spin" << std::endl;
    ros::spinOnce();

    g.acquire(Ic);
    esm.perceive();

    vpDisplay::display(Ic);
    esm.draw(Ic);

    vpDisplay::flush(Ic);
  }

}


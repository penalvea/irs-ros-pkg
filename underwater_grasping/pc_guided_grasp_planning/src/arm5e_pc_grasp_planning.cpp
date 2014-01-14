/*
 * Asks for a 2D grasp, and plans a 3D grasp on the point cloud
 *
 *  Created on: 20/06/2012
 *      Author: mprats
 */
#include <uwsim/ConfigXMLParser.h>

#include <mar_perception/PCGraspPlanning.h>
#include <mar_ros_bridge/mar_params.h>

#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>

#include <uwsim/SceneBuilder.h>
#include <uwsim/ViewBuilder.h>
#include <uwsim/UWSimUtils.h>
#include <uwsim/osgPCDLoader.h>
#include <osgGA/TrackballManipulator>

/** Plans a grasp on a point cloud and visualizes it using UWSim
 * Expects the following params to be set in the ROS parameter server:
 * input_basename: The full path to the input files without extension, ie. /tmp/scan_2
 * resources_data_path: The path the folder with the models for UWSim
 * eMh: a six-element vector representing the hand frame wrt the end-effector frame. [x y z roll pitch yaw] format
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "arm5e_pc_grasp_planning");
	ros::NodeHandle nh;

	std::string input_basename("output");
	nh.getParam("input_basename", input_basename);

	std::string reference_image(input_basename+std::string(".ppm"));
	std::string point_cloud_file(input_basename+std::string(".pcd"));
	std::string camera_params_file(input_basename+std::string(".cam"));

	boost::shared_ptr<vpImage<vpRGBa> > Iorigin(new vpImage<vpRGBa>());
	vpImageIo::read(*Iorigin, reference_image);
	std::cerr << "Image is " << Iorigin->getRows() << "x" << Iorigin->getCols() << std::endl;
	vpDisplayX window(*Iorigin);
	vpDisplay::display(*Iorigin);
	vpDisplay::flush(*Iorigin);

	std::cerr << "Click on the grasp points" << std::endl;
	vpImagePoint ip1,ip2;
	vpDisplay::getClick(*Iorigin,ip1);
	vpDisplay::displayCross(*Iorigin,ip1,15,vpColor::red,2);
	vpDisplay::flush(*Iorigin);
	vpDisplay::getClick(*Iorigin,ip2);
	vpDisplay::displayCross(*Iorigin,ip2,15,vpColor::red,2);
	vpDisplay::displayLine(*Iorigin,ip1,ip2,vpColor::green,2);
	vpDisplay::flush(*Iorigin);

	osgPCDLoader<pcl::PointXYZRGB> pcd_geode(point_cloud_file);
	vpMatrix K_m;
	vpMatrix::loadMatrix(camera_params_file, K_m);
	vpCameraParameters K;
	K.initFromCalibrationMatrix(K_m);
	PCGraspPlanning planner(*Iorigin,ip1,ip2,K,pcd_geode.cloud.makeShared());
	planner.perceive();
	vpDisplay::close(*Iorigin);

	std::cout << "Planned grasp frame with respect to camera is: " << std::endl << planner.get_cMg() << std::endl;
	std::cout << "Starting visualization in UWSim" << std::endl;

	std::string resources_data_path(".");
	nh.getParam("resources_data_path", resources_data_path);
	osgDB::Registry::instance()->getDataFilePathList().push_back(resources_data_path);

	boost::shared_ptr<osg::ArgumentParser> arguments(new osg::ArgumentParser(&argc,argv));
	std::string configfile=resources_data_path+"/arm5e_gripper.xml";
	ConfigFile config(configfile);

	SceneBuilder builder(arguments);
	builder.loadScene(config);

	ViewBuilder view(config, &builder, arguments);

	osg::ref_ptr<osgGA::TrackballManipulator> tb = new osgGA::TrackballManipulator;
	tb->setHomePosition( osg::Vec3f(0,0,0), osg::Vec3f(0,0,-1), osg::Vec3f(-1,0,0) );
	view.getViewer()->setCameraManipulator( tb );
	view.init();
	view.getViewer()->realize();
	view.getViewer()->frame();

	osgViewer::Viewer::Windows windows;
	view.getViewer()->getWindows(windows);
	windows[0]->setWindowName("UWSim");

	UWSimGeometry::applyStateSets(pcd_geode.getGeode());
	builder.getScene()->localizedWorld->addChild(pcd_geode.getGeode());

	vpMatrix cMg=planner.get_cMg().transpose();
	osg::Matrixd osg_cMg(cMg.data);
	osg::MatrixTransform *gt=new osg::MatrixTransform(osg_cMg);
	gt->addChild(UWSimGeometry::createFrame(0.005, 0.1));
	UWSimGeometry::applyStateSets(gt);
	builder.getScene()->localizedWorld->addChild(gt);

	//Hand frame wrt end-effector
	vpHomogeneousMatrix eMh=mar_params::paramToVispHomogeneousMatrix(&nh, "eMh");
	vpHomogeneousMatrix cMe=planner.get_cMg()*eMh.inverse();
	osg::Matrixd osg_cMe(cMe.transpose().data);
	builder.iauvFile[0]->setVehiclePosition(osg_cMe);

	while( ros::ok() && !view.getViewer()->done())
	{
		ros::spinOnce();
		view.getViewer()->frame();
	}

	return 1;
}

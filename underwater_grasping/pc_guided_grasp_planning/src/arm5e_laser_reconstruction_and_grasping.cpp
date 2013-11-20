/*
 * Makes a laser scan, performs a 3D reconstruction, asks for a 2D grasp, and plans a 3D grasp on the point cloud
 *
 *  Created on: 08/06/2012
 *      Author: mprats
 */
#include <ConfigXMLParser.h>

#include <CPerception/VirtualImage.h>
#include <CRobot/ARM5Arm.h>
#include <CPerception/ESMTracking.h>
#include <CPerception/Reconstruction3D.h>
#include <CAction/Reconstruction3DAction.h>
#include <CPerception/MotionEstimator.h>
#include <CPerception/PCGraspPlanning.h>

#include <visp/vpDisplayX.h>

#include <SceneBuilder.h>
#include <ViewBuilder.h>
#include <UWSimUtils.h>
#include <osgPCDLoader.h>
#include <osgGA/TrackballManipulator>


#define SCAN_INITIAL_POSTURE {0, 0.1, 1.4, 0, 0}
#define SCAN_FINAL_POSTURE {0, 0.1, 2.0, 0, 0}

int main(int argc, char **argv) {
	if (argc!=3 && argc!=4 && argc!=5) {
		std::cerr << "Usage: " << argv[0] << " <image_topic> <image_info_topic> [--offline] [--fixed]" << std::endl;
		std::cerr << "Use --offline if the robot arm is not physically connected, e.g. feeding from a bagfile" << std::endl;
		std::cerr << "Use --fixed if the base if fixed and therefore there is no need for motion estimation" << std::endl;
		exit(0);
	}

	std::string image_topic(argv[1]);
	std::string image_info_topic(argv[2]);

	bool offline=false;
	bool fixed=false;

	for (int i=3; i<argc; i++) {
		if (argv[i]==std::string("--offline")) offline=true;
		else if (argv[i]==std::string("--fixed")) fixed=true;
	}

	ros::init(argc, argv, "reconstruction_3d");
	ros::NodeHandle nh;

	ARM5ArmPtr robot(new ARM5Arm(nh));

	VirtualImagePtr grabber(new VirtualImage(nh,image_topic,image_info_topic));

	//Wait for images
	while (!grabber->ready()) {
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
	}

	SimpleSubPixelLaserPeakDetectorPtr peak_detector(new SimpleSubPixelLaserPeakDetector(grabber));
	ArmLaserReconstruction3DPtr rec(new ArmLaserReconstruction3D(peak_detector, robot, tracker, mest, grabber));

	//Camera to arm base calibration
	vpHomogeneousMatrix bMc;
	bMc[0][0]=0; bMc[0][1]=1; bMc[0][2]=0; bMc[0][3]=-0.28;
	bMc[1][0]=-1; bMc[1][1]=0; bMc[1][2]=0; bMc[1][3]=0.08;
	bMc[2][0]=0; bMc[2][1]=0; bMc[2][2]=1; bMc[2][3]=0.05;
	rec->setCameraToBase(bMc);

	vpHomogeneousMatrix eMl=vpHomogeneousMatrix(0,0,0,0,0,-robot->q[3]) * vpHomogeneousMatrix(-0.02,0,0,0,0,0);
	rec->setLaserToEef(eMl);

	Reconstruction3DAction action(I, grabber, robot, tracker, rec, mest);
	action.enableDrawing(boost::shared_ptr<vpImage<vpRGBa> >(I));
	action.setOffline(offline);
	action.setFixedBase(fixed);

	double scan_initial_posture[5]=SCAN_INITIAL_POSTURE;
	double scan_final_posture[5]=SCAN_FINAL_POSTURE;
	vpColVector vp_scan_initial_posture(5);
	vpColVector vp_scan_final_posture(5);
	vp_scan_initial_posture << scan_initial_posture;
	vp_scan_final_posture << scan_final_posture;
	action.setInitialPosture(vp_scan_initial_posture);
	action.setFinalPosture(vp_scan_final_posture);

	action.doAction();

	if (!ros::ok()) return 0;

	*I=*Iorigin;
	vpDisplay::display(*I);
	vpDisplay::flush(*I);

	std::cerr << "Click on the grasp points" << std::endl;
	vpImagePoint ip1,ip2;
	vpDisplay::getClick(*I,ip1);
	vpDisplay::displayCross(*I,ip1,15,vpColor::red,2);
	vpDisplay::flush(*I);
	vpDisplay::getClick(*I,ip2);
	vpDisplay::displayCross(*I,ip2,15,vpColor::red,2);
	vpDisplay::displayLine(*I,ip1,ip2,vpColor::green,2);
	vpDisplay::flush(*I);

	PCGraspPlanning planner(*I,ip1,ip2,grabber->K,rec->getCloud());
	planner.perceive();
	vpDisplay::close(*I);

	std::cout << "Planned grasp frame with respect to camera is: " << std::endl << planner.get_cMg() << std::endl;
	std::cout << "Starting visualization in UWSim" << std::endl;

        osgDB::Registry::instance()->getDataFilePathList().push_back(std::string("."));
        osgDB::Registry::instance()->getDataFilePathList().push_back(std::string("resources/"));

	boost::shared_ptr<osg::ArgumentParser> arguments(new osg::ArgumentParser(&argc,argv));
        string configfile=std::string("arm5e_gripper.xml");
        ConfigFile config(configfile);

	SceneBuilder builder(arguments);
        builder.loadScene(config);
	builder.getScene()->getOceanScene()->setOceanSurfaceHeight(-2.0);

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

	osgPCDLoader<pcl::PointXYZRGB> pcd_geode(*(rec->getCloud()));
	UWSimGeometry::applyStateSets(pcd_geode.getGeode());
	builder.getScene()->localizedWorld->addChild(pcd_geode.getGeode());

	vpMatrix cMg=planner.get_cMg().transpose();
	osg::Matrixd osg_cMg(cMg.data);
	osg::MatrixTransform *gt=new osg::MatrixTransform(osg_cMg);
	gt->addChild(UWSimGeometry::createOSGSphere(0.02));
	builder.getScene()->localizedWorld->addChild(gt);

	//Hand frame wrt end-effector
	vpHomogeneousMatrix eMh(0.25,0,0,0,M_PI_2,0);
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

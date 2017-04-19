#ifndef RECONSTRUCTION3DACTION_H
#define RECONSTRUCTION3DACTION_H

#include <sensor_msgs/PointCloud2.h>

#include <mar_core/CAction.h>

#include <mar_perception/VirtualImage.h>
#include <mar_core/Arm.h>
#include <mar_perception/Reconstruction3D.h>

#include <visp/vpDisplay.h>
#include <visp/vpColVector.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>

/** Maximum joint velocity to send to the arm */
#define DEFAULT_MAX_JOINT_VELOCITY 0.1

/** Stops the scanning when the position error of the joint with the maximum error is below POSITION_TOLERANCE radians */
#define DEFAULT_POSITION_TOLERANCE 0.02

class Reconstruction3DAction : public CAction
{
  typedef enum
  {
    SUCCESS, ERROR
  } output_t;

  //Pointers to perception classes
  ArmPtr robot_;
  std::vector<Reconstruction3DPtr> rec_;
  //ArmLaserReconstruction3DEyePtr rec_eye_;

  vpColVector vp_scan_initial_posture_;
  vpColVector vp_scan_final_posture_;

  double max_joint_velocity_;
  double position_tolerance_;

  bool fixed_base_; ///< If fixed base, do not perform tracking
  bool offline_;	///< If offline, do not try to move the robot arm
  bool done_;
  bool publish_point_cloud_;
  std::vector<int> cont_points_;

  ros::Publisher point_cloud_pub_;
  ros::NodeHandle nh_;
  ros::Subscriber benchmark_sub_;

public:
 /* Reconstruction3DAction(ArmPtr robot, ESMTrackingPtr tracker, MotionEstimatorPtr mest) :
      CAction(), robot_(robot), tracker_(tracker), mest_(mest), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(
          DEFAULT_POSITION_TOLERANCE), fixed_base_(false), offline_(false), done_(false), publish_point_cloud_(false)
  {
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("LaserReconstructionPointCloud", 1000);
    //benchmark_sub_=nh_.subscribe("/BenchmarkInfo", 100, Reconstruction3DAction::benchmarkCallback);
  }*/

  Reconstruction3DAction() :
      CAction(), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(DEFAULT_POSITION_TOLERANCE), fixed_base_(
          true), offline_(true), done_(false), publish_point_cloud_(false)
  {
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("LaserReconstructionPointCloud", 1000);

    //benchmark_sub_=nh_.subscribe("/BenchmarkInfo", 100, Reconstruction3DAction::benchmarkCallback);
  }

 /* Reconstruction3DAction(ESMTrackingPtr tracker, MotionEstimatorPtr mest) :
      CAction(), tracker_(tracker), mest_(mest), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(
          DEFAULT_POSITION_TOLERANCE), fixed_base_(false), offline_(true), done_(false), publish_point_cloud_(false)
  {
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("LaserReconstructionPointCloud", 1000);
    //benchmark_sub_=nh_.subscribe("/BenchmarkInfo", 100, Reconstruction3DAction::benchmarkCallback);
  }*/

  Reconstruction3DAction(ArmPtr robot, std::vector<Reconstruction3DPtr> rec) :
      CAction(), robot_(robot), rec_(rec), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(
          DEFAULT_POSITION_TOLERANCE), fixed_base_(true), offline_(false), done_(false), publish_point_cloud_(false)
  {
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("LaserReconstructionPointCloud", 1000);
    benchmark_sub_ = nh_.subscribe<std_msgs::String>("/BenchmarkInfo", 100, &Reconstruction3DAction::benchmarkCallback,
                                                     this);
    cont_points_.resize(rec_.size(), 0);
  }
  static void threadReconstruction(Reconstruction3DPtr rec, Reconstruction3DAction *act)
  {
    //int contador=0;
    int cont_points = 0;
    while (ros::ok() && !act->done_)
    {

      rec->perceive();
      ros::spinOnce();
      //	contador++;
      if (act->publish_point_cloud_)
      {
        if (cont_points != rec->points3d.size())
        {

          sensor_msgs::PointCloud2 msg;
          /*for(int i=cont_points; i<rec->points3d.size(); i++){
           geometry_msgs::Point32 point;
           point.x=rec->points3d[i][0];
           point.y=rec->points3d[i][1];
           point.z=rec->points3d[i][2];
           msg.points.push_back(point);
           }*/
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
          cloud = rec->getLimitedCloud(cont_points, rec->points3d.size());
          cont_points = rec->points3d.size();

          //act->point_cloud_pub_.publish(*cloud);
          pcl::toROSMsg(*cloud, msg);
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "girona500/kinematic_base";
          act->point_cloud_pub_.publish(msg);
          ros::spinOnce();
        }
      }
    }
    //std::cout<<"Puntos: "<<rec->getCloud()->size()<<std::endl;
    //std::cout<<"Contador thread"<<contador<<std::endl;
  }

  boost::thread createThread(Reconstruction3DPtr rec)
  {
    return boost::thread(&Reconstruction3DAction::threadReconstruction, rec, this);
  }

  int doAction();	///< Perform the action

  virtual void draw();

  /** Set the joint values where to start the scan */
  void setInitialPosture(vpColVector v)
  {
    vp_scan_initial_posture_ = v;
  }

  /** Set the joint values where to finish the scan */
  void setFinalPosture(vpColVector v)
  {
    vp_scan_final_posture_ = v;
  }

  void setMaxJointVelocity(double v)
  {
    max_joint_velocity_ = v;
  }
  void setTolerance(double t)
  {
    position_tolerance_ = t;
  }

  /** If set to true, the action does not perform tracking */
  void setFixedBase(bool flag)
  {
    fixed_base_ = flag;
  }

  /** If set to true, the action does not try to access the robot arm */
  void setOffline(bool flag)
  {
    offline_ = flag;
  }

  /** If set to true, the point cloud is published during the reconstruction */
  void setPointCloudPublished(bool flag)
  {
    publish_point_cloud_ = flag;
  }

  void addReconstruction3D(Reconstruction3DPtr rec)
  {
    rec_.push_back(rec);
  }
  void benchmarkCallback(const std_msgs::String::ConstPtr &msg);

  virtual ~Reconstruction3DAction()
  {
  }
  ;

};

typedef boost::shared_ptr<Reconstruction3DAction> Reconstruction3DActionPtr;

#endif

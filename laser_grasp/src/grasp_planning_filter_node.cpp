#include <laser_grasp/grasp_planning_filter.h>
#include <ros/ros.h>

int main(int argc, char** argv){

  if(argc!=2){
    std::cerr<<"Syntax is: "<<argv[0]<<" main_scene.pcd\n"<<std::endl;
    return -1;
  }

  ros::init(argc, argv, "grasp_planning_filter");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile(argv[1], *cloud)==-1){
    std::cout<<"Couldn't read file "<<std::endl;
  }

  GraspPlanningFilter sym;
  task_priority::ObjectPose_msg msg= sym.publishPose(cloud);




ros::Rate loop_rate(10);
  ros::Publisher pub=nh.advertise<task_priority::ObjectPose_msg>("object_pose", 100);
  while(ros::ok()){
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;


}

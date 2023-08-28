/*
// This program publishes a topic to initiate the target scan
// Tristan Hill - 07/21/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <ros/ros.h>
#include "ros/package.h"
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>

#include "aubo_control/gcodeAction.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;


bool scan_complete=0;
bool cloud_saved=0;

bool target_saved=0;
bool source_saved=0;

bool new_scan;


void scan_state_callback(const std_msgs::Bool::ConstPtr& msg)
{

  if (msg->data){
    ROS_INFO("Scanning...");
  } 
  else if(!msg->data){ 
    ROS_INFO("Not scanning...");
    scan_complete=1;

  }

}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"start_target_scan");
  ros::NodeHandle node;
  ros::Rate loop_rate(5);
  
  // setup subcribers for scan_state and source_out
  ros::Subscriber scan_state_sub = node.subscribe("/cr_weld/scan_state", 1000, scan_state_callback);

  ros::Publisher start_target_scan_pub = node.advertise<std_msgs::Bool> ("/start_target_scan", 1);

  ros::Publisher gcode_action_pub = node.advertise<aubo_control::gcodeAction> ("target_gcode", 1);



  std_msgs::Bool start_target_scan_msg;
  bool start_target_scan;

  aubo_control::gcodeAction gcode_action_msg;



  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     start_target_scan v1.x                                 "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters from config file
  bool save_output, translate_output;
  node.getParam("start_target_scan", new_scan);

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     start_target_scan: publishing start topic      "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;

  start_target_scan=1;
  gcode_action_msg.file_name="scan_target";
  gcode_action_msg.start_job=1;


  //publish forever
  while(ros::ok())
  {

    start_target_scan_msg.data=start_target_scan;
    start_target_scan_pub.publish(start_target_scan_msg);

    gcode_action_pub.publish(gcode_action_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     get_target: complete                            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;
  return 0;
}



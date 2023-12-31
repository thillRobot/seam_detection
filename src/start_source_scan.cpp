/*
// This program publishes a topic to initiate the source scan
// Tristan Hill - 07/21/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <ros/ros.h>
#include "ros/package.h"
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


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

  ros::init(argc,argv,"start_source_scan");
  ros::NodeHandle node;
  ros::Rate loop_rate(5);
  
  // setup subcribers for scan_state and source_out
  ros::Subscriber scan_state_sub = node.subscribe("/cr_weld/scan_state", 1000, scan_state_callback);

  ros::Publisher start_source_scan_pub = node.advertise<std_msgs::Bool> ("/start_source_scan", 1);

  ros::Publisher gcode_action_pub = node.advertise<aubo_control::gcodeAction> ("/gcode_action", 1);

  ros::Publisher gcode_string_pub = node.advertise<std_msgs::String> ("/gcode_string", 1);

  std_msgs::Bool start_source_scan_msg;
  bool start_source_scan;
  bool scan_started;

  aubo_control::gcodeAction gcode_action_msg;
  std_msgs::String gcode_string_msg;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     start_source_scan v1.8                                 "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters from config file
  bool save_output, translate_output;
  node.getParam("new_scan", new_scan); // not used

  start_source_scan=1;
  scan_started=0;
  int idx=0;

  gcode_action_msg.file_name="scan_source";
  gcode_action_msg.start_job=1;

  gcode_string_msg.data="scan_source";

  //publish forever
  while(ros::ok())
  {

    if (!scan_started&&idx<3)
    {
      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"                     start_source_scan: publishing gcode_string     "<<std::endl;
      std::cout<<"===================================================================="<<std::endl;
    


      gcode_action_pub.publish(gcode_action_msg);
      gcode_string_pub.publish(gcode_string_msg);

      //scan_started=1;
      idx++;
    }else if(!scan_started){
      scan_started=1;
    }


    start_source_scan_msg.data=start_source_scan;
    start_source_scan_pub.publish(start_source_scan_msg);


    ros::spinOnce();
    loop_rate.sleep();

  }
  
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     get_source: complete                           "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;
  return 0;
}



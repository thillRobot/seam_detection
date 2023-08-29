/*
// This program gets a set of teach points from the robot arm 
// to use as reference points for the target scan
// Tristan Hill - 08/28/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <ros/ros.h>
#include "ros/package.h"
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;


bool scan_complete=0;
bool cloud_saved=0;

bool target_saved=0;
bool source_saved=0;
bool start_source_scan=0;

bool new_scan;


// global parameters for callback access
std::string output_path, output_file; 

// callback for tool pose simple
void current_pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  //source_saved=msg->data;
  ROS_INFO("current pose: %f", msg->position.x);
}

//callback for teach click
void teach_callback(const std_msgs::Int8::ConstPtr& msg)
{
  //source_saved=msg->data;
  ROS_INFO("teach: %i", msg->data);
}


int main(int argc, char** argv)
{

  ros::init(argc,argv,"teach_points");
  ros::NodeHandle node;
  ros::Rate loop_rate(5);
  
  ros::Subscriber current_pose_sub = node.subscribe("/aubo_robot/current_pose_tool0_basic",10, current_pose_callback);
  ros::Subscriber teach_sub = node.subscribe("/cr_weld/teach_pose",10, teach_callback);

  
  //std_msgs::Bool get_source_state_msg, target_saved_msg, source_saved_msg;
  //get_cloud_state_msg.data=cloud_saved;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points v1.x                              "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points: loading configuration file       "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // no, this is not true, the config file is read in roslaunch, not by the node
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters from config file
  //bool save_output, translate_output;
  //node.getParam("save_output", save_output);
  //node.getParam("translate_output", translate_output);
  //node.getParam("get_source/new_scan", new_scan);
  //node.getParam("get_source/output_file", output_file);
  //output_path=packagepath+'/'+output_file;


  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points: setup complete                   "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;

  //publish forever
  while(ros::ok())
  {
    
    //get_source_state_msg.data=cloud_saved;
    //get_source_state_pub.publish(get_source_state_msg);

    //source_saved_msg.data=cloud_saved;
    //source_saved_pub.publish(source_saved_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points: complete                         "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;
  return 0;
}



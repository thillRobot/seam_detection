/*
// This program gets and saves a pointcloud published by aubo_control main_LIDAR.launch
// Tristan Hill - 07/21/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <ros/ros.h>
#include "ros/package.h"
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;


bool scan_complete=0;
bool cloud_saved=0;

bool target_saved;
bool source_saved;

//bool target_scan=1; // target scan first
//bool source_scan=0; // source scan second 

//bool get_cloud_state=false; 
bool new_scan;

// get additional parameters set in launch file
bool saving_source;
bool saving_target;

// global parameters for callback access
std::string output_path, output_file; 

void scan_state_callback(const std_msgs::Bool::ConstPtr& msg)
{
  //ROS_INFO("I heard scan_state: [%d]", msg->data);
  if (msg->data&&!cloud_saved){
    ROS_INFO("Scanning, waiting to complete ...");
  } 
  if (!msg->data&&!cloud_saved){ 
    ROS_INFO("Scan complete, preparing to save file");
    scan_complete=1;
    ROS_INFO("cloud_saved: %i",cloud_saved);
    ROS_INFO("target_saved: %i",target_saved);
  }
  if (!msg->data&&cloud_saved){
    ROS_INFO("cloud saved previously, skipping");
  }

}

void target_saved_callback(const std_msgs::Bool::ConstPtr& msg)
{
  target_saved=msg->data;
  //ROS_INFO("target_saved: ", target_saved);
}

void source_saved_callback(const std_msgs::Bool::ConstPtr& msg)
{
  source_saved=msg->data;
  //ROS_INFO("source_saved: ", source_saved);
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PointCloud::Ptr cloud_in (new PointCloud);
  pcl::fromROSMsg(*cloud_msg,*cloud_in);
  //ROS_INFO("flag not set, waiting to save");

  if(scan_complete&&!cloud_saved&&saving_target){

    //ROS_INFO("flag set, saving cloud");
    std::cout<<"===================================================================="<<std::endl;
    std::cout<<"                   get_cloud: saving pointcloud data as target      "<<std::endl;
    std::cout<<"===================================================================="<<std::endl<<std::endl;
    // save filtered cloud 
    try{
      pcl::io::savePCDFileASCII (output_path, *cloud_in);
      std::cout<<"Cloud saved to: "<< output_path <<std::endl;
      target_saved=1;
      cloud_saved=1;
 
    }catch(...){
      std::cout<<"Cloud not saved."<<std::endl;
    } 

  }else if(scan_complete&&!cloud_saved&&target_saved&&saving_source){

    //ROS_INFO("flag set, saving cloud");
    std::cout<<"===================================================================="<<std::endl;
    std::cout<<"                   get_cloud: saving pointcloud data as source      "<<std::endl;
    std::cout<<"===================================================================="<<std::endl<<std::endl;
    // save filtered cloud 
    try{
      pcl::io::savePCDFileASCII (output_path, *cloud_in);
      std::cout<<"Cloud saved to: "<< output_path <<std::endl;
      source_saved=1;
      cloud_saved=1;
 
    }catch(...){
      std::cout<<"Cloud not saved."<<std::endl;
    }
  }
 
  //ros::spinOnce();
}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"get_cloud");
  ros::NodeHandle node;
  ros::Rate loop_rate(5);
  
  // setup subcribers for scan_state and cloud_out
  ros::Subscriber scan_state_sub = node.subscribe("/cr_weld/scan_state", 1000, scan_state_callback);
  ros::Subscriber cloud_sub = node.subscribe("/cloud_out",10, cloud_callback);
  ros::Subscriber target_saved_sub = node.subscribe("/get_cloud/target_saved",10, target_saved_callback);
  ros::Subscriber source_saved_sub = node.subscribe("/get_cloud/source_saved",10, source_saved_callback);

  // publisher for save_cloud_state, target_saved, source_saved
  ros::Publisher get_cloud_state_pub = node.advertise<std_msgs::Bool> ("/get_cloud/get_cloud_state", 1);
  ros::Publisher target_saved_pub = node.advertise<std_msgs::Bool> ("/get_cloud/target_saved", 1);
  ros::Publisher source_saved_pub = node.advertise<std_msgs::Bool> ("/get_cloud/source_saved", 1);
  
  std_msgs::Bool get_cloud_state_msg, target_saved_msg, source_saved_msg;
  //get_cloud_state_msg.data=cloud_saved;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     get_cloud v1.8                                 "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     get_cloud: loading configuration file          "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // no, this is not true, the config file is read in roslaunch, not by the node
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters from config file
  bool save_output, translate_output;
  node.getParam("save_output", save_output);
  node.getParam("translate_output", translate_output);
  node.getParam("get_cloud/new_scan", new_scan);
  node.getParam("get_cloud/output_file", output_file);
  output_path=packagepath+'/'+output_file;

  // get additional parameters set in launch file
  //bool saving_source=0; // moved to global scope
  //bool saving_target=0;
  
  //get flags saving_target, saving_source as command line args (not from config file)
  if (argc>1){
    std::string arg1(argv[1]);    
    if (!arg1.compare("saving_target")){
      saving_target=1;
    }
    if (!arg1.compare("saving_source")){
      saving_source=1;
    }
    std::cout<<"saving_target set: "<<saving_target<<" through cmd line"<<std::endl;
    std::cout<<"saving_source set: "<<saving_source<<" through cmd line"<<std::endl;
  }

  // by pass wait for new scan
  if(!new_scan){
    std::cout<<"Using previous scan from file: "<< output_path <<std::endl;
    cloud_saved=1;
  }

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     get_cloud: loading pointcloud data             "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;

  //publish forever
  while(ros::ok())
  {
    
    get_cloud_state_msg.data=cloud_saved;
    get_cloud_state_pub.publish(get_cloud_state_msg);

    target_saved_msg.data=target_saved;
    target_saved_pub.publish(target_saved_msg);

    source_saved_msg.data=source_saved;
    source_saved_pub.publish(source_saved_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     get_cloud: complete                            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;
  return 0;
}



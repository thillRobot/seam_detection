/*
// This program gets and saves a pointcloud published by aubo_control main_LIDAR.launch
// Tristan Hill - 07/21/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>

#include <pcl/pcl_config.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> 

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
 
#include "ros/package.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
//#include <tf/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;

bool scan_complete=0;
bool cloud_saved=0;
//bool get_cloud_state=false; 
bool new_scan;

// global parameters for callback access
std::string output_path, output_file; 

void scan_stateCallback(const std_msgs::Bool::ConstPtr& msg)
{
  //ROS_INFO("I heard scan_state: [%d]", msg->data);
  if (msg->data){
    ROS_INFO("Scan beginning, waiting to complete ...");
  }
  else if (!msg->data){
    ROS_INFO("Scan complete, preparing to save file");
    scan_complete=1;
  }
}

void cloud_Callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PointCloud::Ptr cloud_in (new PointCloud);
  pcl::fromROSMsg(*cloud_msg,*cloud_in);
  //ROS_INFO("flag not set, waiting to save");

  if(scan_complete&&!cloud_saved){

    //ROS_INFO("flag set, saving cloud");
    std::cout<<"===================================================================="<<endl;
    std::cout<<"                   get_cloud: saving pointcloud data                "<<endl;
    std::cout<<"===================================================================="<<endl<<endl;
    // save filtered cloud 
    try{
      pcl::io::savePCDFileASCII (output_path, *cloud_in);
      std::cout<<"Cloud saved to: "<< output_path <<std::endl;
      cloud_saved=1;
    }catch(...){
      std::cout<<"Cloud not saved."<<std::endl;
    }
  } 

  ros::spinOnce();
}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"get_cloud");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);
  
  // setup subcribers for scan_state and cloud_out
  ros::Subscriber scan_state_sub = node.subscribe("/cr_weld/scan_state", 1000, scan_stateCallback);
  ros::Subscriber cloud_sub = node.subscribe("/cloud_out",10, cloud_Callback);

  // publisher for save_cloud_state
  ros::Publisher get_cloud_state_pub = node.advertise<std_msgs::Bool> ("/get_cloud/get_cloud_state", 1);
  
  std_msgs::Bool get_cloud_state_msg;
  get_cloud_state_msg.data=cloud_saved;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     get_cloud v1.x                                 "<<endl;
  std::cout<<"===================================================================="<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     get_cloud: loading configuration file          "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // there is only one cmd line arg and it is the name of the config file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters 
  bool save_output, translate_output;
  node.getParam("save_output", save_output);
  node.getParam("translate_output", translate_output);
  node.getParam("get_cloud/new_scan", new_scan);

  node.getParam("get_cloud/output_file", output_file);
  output_path=packagepath+'/'+output_file;

  // by pass wait for new scan
  if(!new_scan){
    std::cout<<"Using previous scan from file: "<< output_path <<std::endl;
    cloud_saved=1;
  }

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     get_cloud: loading pointcloud data             "<<endl;
  std::cout<<"===================================================================="<<endl;

  //publish forever
  while(ros::ok())
  {
    get_cloud_state_msg.data=cloud_saved;
    get_cloud_state_pub.publish(get_cloud_state_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     get_cloud: complete                            "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;
  return 0;
}



/*
// This program applies a series of filters and processes to the input pointcloud
// to prepare for registration 
// A .pcd file is read as input and a new .pcd file is written as output
// A target .pcd file can also be input for cluster selection
// Tristan Hill - 07/14/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation

// this code is based on PCL example code: 
// https://pcl.readthedocs.io/en/latest/passthrough.html
// https://pcl.readthedocs.io/en/latest/cluster_extraction.html

// PCA box example code algorithm adapted from:
// http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
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

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
//#include <tf/TransformStamped.h>

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
//#include <teaser/point_cloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;

bool get_cloud_complete=0;
bool filtered_cloud_saved=0;

void get_cloud_stateCallback(const std_msgs::Bool::ConstPtr& msg)
{
  //ROS_INFO("I heard scan_state: [%d]", msg->data);
  if (!msg->data){
    ROS_INFO("get_cloud in progress, waiting to begin filtering ...");
  }
  else if (msg->data&&!get_cloud_complete){
    ROS_INFO("get_cloud complete, beginning filtering");
    get_cloud_complete=1;
  }
}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"rotate_cloud");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);
  
  // setup subcribers for get_cloud_state
  //ros::Subscriber get_cloud_state_sub = node.subscribe("/get_cloud/get_cloud_state", 1000, get_cloud_stateCallback);

  // publisher for filter_cloud_state
  //ros::Publisher filter_cloud_state_pub = node.advertise<std_msgs::Bool> ("/filter_cloud/filter_cloud_state", 1);
  
  std_msgs::Bool filter_cloud_state_msg;
  filter_cloud_state_msg.data=filtered_cloud_saved;


  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     rotate_cloud v1.x                              "<<endl;
  std::cout<<"===================================================================="<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     rotate_cloud: loading configuration file                    "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // there is only one cmd line arg and it is the name of the config file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters 
  bool save_output;
  node.getParam("rotate_cloud/save_output", save_output);

  // parameters that contain strings  
  std::string input_path, output_path, target_path, input_file, output_file, target_file; 
  node.getParam("rotate_cloud/input_file", input_file);
  input_path=packagepath+'/'+input_file;
  node.getParam("rotate_cloud/output_file", output_file);
  output_path=packagepath+'/'+output_file;

  std::cout<<"Debug0"<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     filter_cloud: preparing pointcloud data        "<<endl;
  std::cout<<"===================================================================="<<endl;
  
  // instantiate some cloud pointers
  PointCloud::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZ>);

  std::cout << "Loading cloud files:" <<std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, *cloud_input) == -1)
  {
      std::cout<<"Couldn't read cloud_input file:"<<input_path;
      return (-1);
  }
  std::cout << "Loaded input cloud file: "<< input_path <<std::endl<<
    cloud_input->width * cloud_input->height << " Data points from "<< input_path << std::endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    rotate_cloud: processing pointcloud data        "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // Rotate pointcloud by alpha, beta, gamma about z,y,x
  // set rotation and origin of a quaternion for the tf transform object
  double alpha, beta, gamma, dtr;
  dtr=M_PI/180.0;

  // rotation angles for yaw pitch roll
  alpha=0*dtr;beta=0*dtr;gamma=90*dtr; 

  // rotation matrix for Yaw Pitch Roll by alpha gamma beta
  tf::Matrix3x3 R_zyx(cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma),
                       sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma),
                       -sin(beta)          , cos(beta)*sin(gamma)                                 , cos(beta)*cos(gamma));  

  // quaternion for previous rotation matrix
  tf::Quaternion q_zyx;
  R_zyx.getRotation(q_zyx); // sets quaternion q_intr with rotation from R_intr (returns normalized quaternion?, check on this)

  tf::StampedTransform *T_zyx (new tf::StampedTransform); 
  T_zyx->setRotation(q_zyx);
  T_zyx->setOrigin(tf::Vector3(0, 0, 0)); // no translation component of the transformation (is 0,0,0 default?)
  // need to normalize quaternion here?

  // transform source cloud to ith intermediate starting position 
  pcl_ros::transformPointCloud(*cloud_input, *cloud_rotated, *T_zyx);


  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    rotate_cloud: saving pointcloud data            "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;


  // save rotated cloud 
  if(save_output){
    pcl::io::savePCDFileASCII (output_path, *cloud_rotated);
    std::cout<<"Rotated cloud written to:"<< output_path <<std::endl;
  }

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   rotate_cloud: pointcloud processing complete     "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   rotate_cloud: preparing visualization            "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  ros::Publisher pub_input = node.advertise<PointCloud> ("/cloud_input", 1) ;
  ros::Publisher pub_rotated = node.advertise<PointCloud> ("/cloud_rotated", 1) ;

  cloud_input->header.frame_id = "base_link";
  cloud_rotated->header.frame_id = "base_link";

    
  std::cout<<"===================================================================="<<endl;
  std::cout<<"                        rotate_cloud: complete                     "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  //publish forever
  while(ros::ok())
  {

      pub_input.publish(cloud_input);
      pub_rotated.publish(cloud_rotated);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}



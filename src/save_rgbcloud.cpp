
// program to subscribe to a pointcloud and save it as a pcd file

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_ros/point_cloud.h>

#include "ros/package.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

//#include <cloudutils.h>
//#include <cloudregistration.h>
#include <cloudfilter.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL XYZ RGB Pointclouds type definitions
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

geometry_msgs::Pose current_pose;
geometry_msgs::PoseArray teach_poses;


// global variable for the output file path, to be used by callback
std::string output_path;
bool file_saved=0;

std::vector<double> bounding_box;
double voxel_size;

void filter_cloud(PointCloud& cloud);

// callback function for recieving cloud messages
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	PointCloud::Ptr cloud_in (new PointCloud);
	pcl::fromROSMsg(*cloud_msg,*cloud_in);  // copy from ROS msg to pcl pointcloud

  std::cout<<"Callback fired"<<std::endl;

  filter_cloud(*cloud_in);

	//ROS_INFO("flag set, saving cloud");
	std::cout<<"===================================================================="<<std::endl;
	std::cout<<"                   save_rgbcloud: saving pointcloud as pcd file     "<<std::endl;
	std::cout<<"===================================================================="<<std::endl<<std::endl;
	// save filtered cloud 
	if(!file_saved){
    try{
  	  pcl::io::savePCDFileASCII (output_path, *cloud_in);
  	  std::cout<<"Cloud saved to: "<< output_path <<std::endl;
      file_saved=1;
  	}catch(...){
  	  std::cout<<"Cloud not saved."<<std::endl;
  	}
  }

}


void filter_cloud(PointCloud &cloud_in){

  PointCloud::Ptr cloud (new PointCloud);
  pcl::copyPointCloud(cloud_in, *cloud);

  //tf::TransformListener listener;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  std::string base="base_link";
  std::string frame="camera_link";

  geometry_msgs::TransformStamped tf_frame_base;
  Eigen::Vector3d frame_base_translation;
  Eigen::Quaterniond frame_base_rotation;

  int k=0;
  while (k<10){ // try 10 times

    try{
      
      // lookup a transform
      tf_frame_base = tfBuffer.lookupTransform(base, frame, ros::Time(0)); 
   
      // convert tranform components to eigen objects to be used in pointcloud transformation 
      tf::vectorMsgToEigen(tf_frame_base.transform.translation, frame_base_translation);
      tf::quaternionMsgToEigen(tf_frame_base.transform.rotation, frame_base_rotation); 
    
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.1).sleep();
      //continue;
    }  

    k++;
  } 

  current_pose.position.x=tf_frame_base.transform.translation.x;
  current_pose.position.y=tf_frame_base.transform.translation.y;
  current_pose.position.z=tf_frame_base.transform.translation.z;

  current_pose.orientation.x=tf_frame_base.transform.rotation.x;
  current_pose.orientation.y=tf_frame_base.transform.rotation.y;
  current_pose.orientation.z=tf_frame_base.transform.rotation.z;
  current_pose.orientation.w=tf_frame_base.transform.rotation.w;

  std::cout<<"camera_link position:[ "<< current_pose.position.x <<","
                                << current_pose.position.y <<","
                                << current_pose.position.z <<"]"<< std::endl; 
  std::cout<<"camera_link orientation:[ "<< current_pose.orientation.x <<","
                                   << current_pose.orientation.y <<","
                                   << current_pose.orientation.z <<","
                                   << current_pose.orientation.w <<"]"<< std::endl; 


  CloudFilter filter;
  filter.loadConfig("save_rgbcloud");
  bool transforming=1;
  bool bounding=1;
  bool smoothing=1;
  bool downsampling=1;

  // apply series of filters to pointcloud
  //rigid transformation
  if(transforming){
    //transformCloud(*cloud, *cloud, base_camera_rotation, base_camera_translation); // inverse
    //transformCloud(*cloud, *cloud, camera_base_rotation, camera_base_translation);
    filter.transformCloud(*cloud, *cloud, frame_base_rotation, frame_base_translation);
  }
  // bounding box 
  if(bounding){ 
    //boundCloud(*cloud, *cloud, bounding_box);
    filter.boundCloud(*cloud, *cloud, bounding_box);
  }

  // voxel downsampling  
  if(downsampling){
    //downsampleCloud(*cloud, *cloud, voxel_size);
    filter.downsampleCloud(*cloud, *cloud, voxel_size);
  }          

}
 

int main(int argc, char** argv)
{

  ros::init(argc,argv,"save_rgbcloud");
  ros::NodeHandle node;
  ros::Rate loop_rate(10);

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     save_rgbcloud v1.9                            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                      Loading Configuration File                    "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // find the path to the this package (seam_detection)
  std::string package_path = ros::package::getPath("seam_detection");


  node.getParam("bounding_box",  bounding_box);
  node.getParam("voxel_size", voxel_size);

  // get additional command line parameters (passed through launch file)  
  if (argc<3){
    std::cout<<"error: argc<2, input_topic(argv[1]) and output_file(argv[2]) must be set in cmd line"<<std::endl;
  	return -1; // quit main early with a neg flag... i dont really like this here but it works
  }

  // instatiate constant strings from cmd line args
  const std::string input_topic(argv[1]);    
  const std::string output_file(argv[2]);    
  
  output_path=package_path+"/"+output_file;

  std::cout<<"input_cloud set: "<<input_topic<<" through cmd line"<<std::endl;
  std::cout<<"output_file set: "<<output_file<<" through cmd line"<<std::endl;
  
  ros::Subscriber cloud_sub = node.subscribe(input_topic, 10, cloud_callback);
 
  


  int count=0;
  // loop for a low number of iterations then quit
  while(ros::ok()&&!file_saved){
    ros::spinOnce();    
    loop_rate.sleep();
  	//std::cout<<"main loop count: "<<count<<std::endl;
  	count++;
  }

}
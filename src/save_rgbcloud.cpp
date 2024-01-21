
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


// PCL XYZ RGB Pointclouds type definitions
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

std::string output_path;


void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	PointCloud::Ptr cloud_in (new PointCloud);
	pcl::fromROSMsg(*cloud_msg,*cloud_in);  // copy from ROS msg to pcl pointcloud

	//ROS_INFO("flag set, saving cloud");
	std::cout<<"===================================================================="<<std::endl;
	std::cout<<"                   save_rgbcloud: saving pointcloud as pcd file     "<<std::endl;
	std::cout<<"===================================================================="<<std::endl<<std::endl;
	// save filtered cloud 
	try{
	  pcl::io::savePCDFileASCII (output_path, *cloud_in);
	  std::cout<<"Cloud saved to: "<< output_path <<std::endl;
	}catch(...){
	  std::cout<<"Cloud not saved."<<std::endl;
	}
}
 

int main(int argc, char** argv)
{

  ros::init(argc,argv,"save_rgbcloud");
  ros::NodeHandle node;
  ros::Rate loop_rate(10);

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     save_rgbcloud v1.8                            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                      Loading Configuration File                    "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // find the path to the this package (seam_detection)
  std::string package_path = ros::package::getPath("seam_detection");

  // get additional command line parameters (passed through launch file)  
  if (argc<2){
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
  // loop for some iterations
  while(ros::ok()&&count<5){
    ros::spinOnce();    
    loop_rate.sleep();
  	std::cout<<"main loop count: "<<count<<std::endl;
  	count++;
  }

}
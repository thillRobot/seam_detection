/*

 cloudutils - utility functions for working with pointsclouds, PCL, and ROS
 Tristan Hill - 03/03/2024
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/


#include "cloudutils.h"
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry> 

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
// PCL PointCloud with XYZ RGB Normal Points
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

// DEFINITIONS

// default constructor

CloudUtils::CloudUtils() 
  : config("cloudutils"){
  
  // find the path to the this package (seam_detection)
  package_path = ros::package::getPath("seam_detection");

}


CloudUtils::CloudUtils(std::string cfg="cloudutils") 
  : config(cfg){
}

CloudUtils::~CloudUtils(){
}


void CloudUtils::loadConfig(std::string cfg){

  config=cfg;
  
  node.getParam("input_file", input_file);    
  node.getParam("output_file", output_file);
 
} 

std::string CloudUtils::getConfig(void){

  return config;

}


// templated function to load pcl::PointCloud<point_t> from PCD file as defined in config
template <typename point_t>
int CloudUtils::loadCloud(pcl::PointCloud<point_t> &input, std::string file) {

  std::cout<<"|---------- CloudUtils::LoadCloud - loading PCD file ----------|"<<std::endl;

  std::string path;
  path=package_path+"/"+file;
  //path=file;

  std::cout << "Loading input pointcloud file: " << path << std::endl;
  if (pcl::io::loadPCDFile<point_t> (path, input) == -1)
  {
    std::cout<<"Failed to load input pointcloud file: "<< path <<std::endl;
    return (-1);
  }
  std::cout << "Loaded "<<input.width * input.height << " data points from input pointcloud file: "<< path <<std::endl;
  return 0;

}

template int CloudUtils::loadCloud<pcl::PointXYZRGB>
              (pcl::PointCloud<pcl::PointXYZRGB> &input, std::string file);







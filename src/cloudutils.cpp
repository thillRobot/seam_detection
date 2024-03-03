/*

 cloudutils - utility functions for working with pointsclouds, PCL, and ROS
 Tristan Hill - 03/03/2024
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/

#include "cloudutils.h"
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

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
  : config("cloudutils"), pub_idx(0){
  
  // find the path to the this package (seam_detection)
  package_path = ros::package::getPath("seam_detection");

}


CloudUtils::CloudUtils(std::string cfg="cloudutils", int idx=0) 
  : config(cfg), pub_idx(idx){
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


// templated function to publish a single pcl::PointCloud<point_t> as a ROS topic 
template <typename point_t>
void CloudUtils::publishCloud(pcl::PointCloud<point_t> &cloud, std::string topic, std::string frame){
  std::cout<<"|---------- SeamDetection::publishCloud - publishing single cloud ----------|"<<std::endl;

  // advertise a new topic and publish a msg each time this function is called
  pub_clouds.push_back(node.advertise<pcl::PointCloud<point_t>>(topic, 0, true));

  cloud.header.frame_id = frame; // reference to show cloud in rviz

  pub_clouds[pub_clouds.size()-1].publish(cloud);

  ros::spinOnce();

}

// declare all possible types used with the template here 
template void CloudUtils::publishCloud< pcl::PointXYZRGB >
              (pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string topic, std::string frame);

template void CloudUtils::publishCloud< pcl::PointXYZRGBNormal >
              (pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, std::string topic, std::string frame);


// function to publish a vector of PointClouds representing clusters as a ROS topic
void CloudUtils::publishClusters(PointCloudVec &clusters, std::string prefix){
  std::cout<<"|---------- CloudUtils::publishClusters - publishing clusters ----------|"<<std::endl;
  std::cout<<"|---------- overloaded for `PointCloudVec` ----------|"<<std::endl;

  std::cout<<"clusters size: "<<clusters.size()<<std::endl;

  for (int i=0; i<clusters.size(); i++){
    // advertise a topic and publish a msg for each cluster in clusters
    std::stringstream name;
    name << prefix << i;
    pub_clusters.push_back(node.advertise<PointCloud>(name.str(), 0, true));
    clusters[i]->header.frame_id = "base_link";
    pub_clusters[pub_idx].publish(clusters[i]);
    pub_idx++;
  }

  ros::spinOnce();
}


// overloaded function to publish a vector of PointClouds with normals representing clusters as a ROS topic
void CloudUtils::publishClusters(PointCloudNormalVec &clusters, std::string prefix){
  std::cout<<"|---------- CloudUtils::publishClusters - publishing clusters ----------|"<<std::endl;
  std::cout<<"|---------- overloaded for `PointCloudNormalVec` ----------|"<<std::endl;

  for (int i=0; i<clusters.size(); i++){
    // advertise a topic and publish a msg for each cluster in clusters
    std::stringstream name;
    name << prefix << i;
    pub_clusters.push_back(node.advertise<PointCloudNormal>(name.str(), 0, true));
    clusters[i]->header.frame_id = "base_link";
    pub_clusters[pub_idx].publish(clusters[i]);
    pub_idx++;
  }

  ros::spinOnce();
}


// templated function to publish a vector of PointClouds with normals representing clusters as a ROS topic
template <typename point_t>
void CloudUtils::publishClusters(const std::vector<typename pcl::PointCloud<point_t>::Ptr,
        Eigen::aligned_allocator<typename pcl::PointCloud<point_t>::Ptr> > &clusters, std::string prefix){
  std::cout<<"|---------- CloudUtils::publishClusters - publishing clusters ----------|"<<std::endl;
  std::cout<<"|---------- templated for `PointCloudVec<pcl::PointCloud<point_t>>` ----------|"<<std::endl;

  for (int i=0; i<clusters.size(); i++){
    // advertise a topic and publish a msg for each cluster in clusters
    std::stringstream name;
    name << prefix << i;
    pub_clusters.push_back(node.advertise<pcl::PointCloud<point_t>>(name.str(), 0, true)); // this type needs handling too
    clusters[i]->header.frame_id = "base_link";
    pub_clusters[pub_idx].publish(clusters[i]);
    pub_idx++;
  }

  ros::spinOnce();
}

template void CloudUtils::publishClusters<pcl::PointXYZRGB>
              (const std::vector<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               Eigen::aligned_allocator<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > &clusters, std::string prefix);



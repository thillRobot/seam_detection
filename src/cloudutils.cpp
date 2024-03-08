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


// templated function to save pcl::PointCloud<point_t> to PCD file as defined in config
template <typename point_t>
int CloudUtils::saveCloud(pcl::PointCloud<point_t> &output, std::string file){
  // the name output here is very confusing, consider changing this
  std::cout<<"|---------- FilterDataset::SaveCloud - saving PCD file ----------|"<<std::endl;

  std::string path;
  path=package_path+"/"+file;
  //path=file;
  // save filtered cloud 

  std::cout << "Saving output pointcloud file: " << path << std::endl;
  if (  pcl::io::savePCDFileASCII(path, output) == -1)
  {
    std::cout<<"Failed to save ouput pointcloud file: "<< path <<std::endl;
    return (-1);
  }
  std::cout << "Saved "<<output.width * output.height << " data points to output pointcloud file: "<< path <<std::endl;
  return 0;
}
 
template int CloudUtils::saveCloud<pcl::PointXYZRGB>
            ( pcl::PointCloud<pcl::PointXYZRGB> &output, std::string file );   


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
void CloudUtils::publishClustersT(const std::vector<typename pcl::PointCloud<point_t>::Ptr,
        Eigen::aligned_allocator<typename pcl::PointCloud<point_t>::Ptr> > &clusters, std::string prefix){
  std::cout<<"|---------- CloudUtils::publishClustersT - publishing clusters ----------|"<<std::endl;
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

template void CloudUtils::publishClustersT<pcl::PointXYZRGB>
              (const std::vector<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               Eigen::aligned_allocator<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > &clusters, std::string prefix);


// function to copy PointCloud with XYZRGB points - not needed, use pcl::copyPointCloud()
void CloudUtils::copyCloud(PointCloud &input, PointCloud &output){

  std::cout<<"the point cloud input has "<< input.size()<< " points"<<std::endl;
  for (int i=0; i<input.size(); i++) { // add points to cluster cloud
    output.push_back(input[i]);
  }
  std::cout<<"the point cloud output has "<< output.size()<< " points"<<std::endl;

}


// function to return the median value of a std::vector
// it seems like there would be a std method for this
double CloudUtils::getMedian(std::vector<double> vals){

  size_t size=vals.size();

  if (size==0){
    return 0; // size 0 vector has no median
  }else{
    std::sort(vals.begin(), vals.end());
    if(size%2==0){
      return (vals[size/2-1]+vals[size/2])/2;
    }else{
      return vals[size/2];
    }
  }
}


// overloaded function to return the median value of a Eigen::VectorXd (dynxamic sized vector of doubles)
double CloudUtils::getMedian(Eigen::VectorXd vals){

  int size=vals.size();

  if (size==0){
    return 0; // size 0 vector has no median
  }else{
    std::sort(vals.data(), vals.data()+vals.size());
    if(size%2==0){
      return (vals[size/2-1]+vals[size/2])/2;
    }else{
      return vals[size/2];
    }
  }
}

/*
// function to return the median value of a std::vector
// it seems like there would be a std method for this
std::vector<double> CloudUtils::getMedianColor(pcl::PointCloud<pcl::PointXYZRGB> &input ){

  //size_t size=vals.size();

  // term 4 - color metric
  //std::uint32_t red, green, blue;
  //int i = 1;
  double red_med, green_med, blue_med;
  //red=input.points[10].r;
  //blue=input.points[10].g;
  //green=input.points[10].b;

  int size=input.size();
  
  std::vector<double> reds, blues, greens;

  
  for (int i; i<=size; i++){
    reds.push_back(input.points[10].r);
    greens.push_back(input.points[10].g);
    blues.push_back(input.points[10].b);
    
  }
  int k=11;
  //std::cout<<"cloud [red, green ,blue] values : ["
  //         <<reds[k]<<", "<<greens[k]<<", "<<blues[k]<<" ]"<<std::endl; 
  
  red_med=getMedian(reds);
  green_med=getMedian(greens);
  blue_med=getMedian(blues);

  std::vector<double> rgb_meds;
  rgb_meds.push_back(red_med);
  rgb_meds.push_back(green_med);
  rgb_meds.push_back(blue_med);

  //std::cout<<"cloud [red, green ,blue] median values : ["
  //         <<red_med<<", "<<green_med<<", "<<blue_med<<" ]"<<std::endl;
  return rgb_meds;
}
*/


// overloaded function to return the median rgb values of a pcl::PointCloud<pcl::PointXYZRGB>
// this returns an Eigen::VectorXd containing the three median color vals
Eigen::VectorXd CloudUtils::getMedianColor(pcl::PointCloud<pcl::PointXYZRGB> &input ){

  //size_t size=vals.size();

  double red_med, green_med, blue_med;

  int size=input.size();
  
  std::vector<double> reds, blues, greens;

  for (int i=0; i<=size; i++){
    reds.push_back(input.points[i].r);
    greens.push_back(input.points[i].g);
    blues.push_back(input.points[i].b);   
  }
  
  red_med=getMedian(reds);
  green_med=getMedian(greens);
  blue_med=getMedian(blues);
  
  Eigen::VectorXd rgb_meds(3);
  rgb_meds(0)=red_med;
  rgb_meds(1)=green_med;
  rgb_meds(2)=blue_med;
  
  std::cout<<"cloud [red, green ,blue] median values : ["
           <<rgb_meds[0]<<", "<<rgb_meds[1]<<", "<<rgb_meds[2]<<" ]"<<std::endl;
  
  return rgb_meds;
}


// function to merge a vector of pointclouds into a single pointcloud
void CloudUtils::mergeClusters(PointCloudVec &clusters, PointCloud &output){

  for (int i=0; i<clusters.size(); i++){
  
    for (int j=0; j<clusters[i]->size(); j++){
      output.push_back(clusters[i]->points[j]);
    }
  
  }

  std::cout<< "the merged cloud has "<< output.size() << " points" <<std::endl;
}



// overloaded function to merge a vector of pointclouds and return pointer to single pointcloud 
PointCloud::Ptr CloudUtils::mergeClusters(PointCloudVec &clusters){

  PointCloud::Ptr output (new PointCloud);

  for (int i=0; i<clusters.size(); i++){
  
    for (int j=0; j<clusters[i]->size(); j++){
      output->push_back(clusters[i]->points[j]);
    }
  
  }

  std::cout<< "the merged cloud has "<< output->size() << " points" <<std::endl;
  return output;
}





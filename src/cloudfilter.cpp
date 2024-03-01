/*

 cloudfilter - applies a series of filters and processes to the input pointcloud
 Tristan Hill - 02/29/2024
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/


#include "cloudfilter.h"
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

// DEFINITIONS


// default constructor

CloudFilter::CloudFilter() 
  : config("cloudfilter"){
}

CloudFilter::CloudFilter(std::string cfg="cloudfilter") 
  : config(cfg){
}

CloudFilter::~CloudFilter(){
}


void CloudFilter::loadConfig(std::string cfg){

  config=cfg;
  
  node.getParam("input_file", input_file);    
  node.getParam("output_file", output_file);
  node.getParam("bounding_box",  bounding_box);
  node.getParam("auto_bounds",  auto_bounds);

} 

std::string CloudFilter::getConfig(void){

  return config;

}

std::vector<double> CloudFilter::getBoundingBox(void){
 
  return bounding_box;

}


// apply bounding box to PointCloud with XYZRGB points
void CloudFilter::boundCloud(PointCloud &input, PointCloud &output, std::vector<double> box){

  PointCloud::Ptr cloud (new PointCloud);      // working copy for this routine
  for (int i=0; i<input.size(); i++) { // add points to cluster cloud
    cloud->push_back(input[i]);  
  } 

  std::cout<<"Beginning BoundCloud() function" << std::endl;
 
  double box_length, box_width, box_height;
  box_length=0.25; // default auto_bounds, smart auto bounds not implemented
  box_width=0.25;     
  box_height=0.25;
  /*
  if (auto_bounds){

    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;  

    pcl::compute3DCentroid(*cloud, centroid);

    box[0]=centroid[0]-box_length/2;  // xmin
    box[1]=centroid[0]+box_length/2;  // xmax
    box[2]=centroid[1]-box_width/2;   // ymin
    box[3]=centroid[1]+box_width/2;   // ymax
    box[4]=centroid[2]-box_height/2;  // zmin
    box[5]=centroid[2]+box_height/2;  // zmax

  }*/
 
  //Apply Bounding Box Filter
  pcl::PassThrough<PointT> pass; //input_cloud
  
  std::cout<<"--- bound cloud debug --- "<<std::endl;
  pass.setInputCloud(cloud);

  std::cout<<"--- bounding_box --- "<<std::endl
             <<"X["<<box[0]<<","<<box[1]<<"]"<<std::endl
             <<"Y["<<box[2]<<","<<box[3]<<"]"<<std::endl
             <<"Z["<<box[4]<<","<<box[5]<<"]"<<std::endl;

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(box[0],box[1]);
  pass.filter (*cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(box[2],box[3]);
  pass.filter (*cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits(box[4],box[5]);
  pass.filter (*cloud);
    
  std::cout<<"--- bound cloud debug --- "<<std::endl;
  pcl::copyPointCloud(*cloud, output);
  std::cout<<"after bounding there are "<<output.size()<<"points in the cloud"<<std::endl;
}


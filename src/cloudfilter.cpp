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
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
 
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry> 

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
// PCL PointCloud with XYZ RGB Normal Points
//typedef pcl::PointXYZRGBNormal PointNT;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

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


// apply translation and rotation without scaling to PointCloud
void CloudFilter::transformCloud(PointCloud &input, PointCloud &output, Eigen::Vector3d rotation, Eigen::Vector3d translation){

  PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy of the training cloud
  pcl::copyPointCloud(input,*cloud);

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  // Define a translation 
  transform.translation() << translation[0], translation[1], translation[2];
  // define three axis rotation&clouds (RPY)
  transform.rotate (Eigen::AngleAxisd (rotation[0], Eigen::Vector3d::UnitX()));
  transform.rotate (Eigen::AngleAxisd (rotation[1], Eigen::Vector3d::UnitY()));
  transform.rotate (Eigen::AngleAxisd (rotation[2], Eigen::Vector3d::UnitZ()));

  // Print the transformation
  //std::cout << transform_2.matrix() << std::endl;

  // Execute the transformation on working copy 
  pcl::transformPointCloud (*cloud, *cloud, transform); 
  // copy to the output cloud
  pcl::copyPointCloud(*cloud, output);
  
  std::cout<<"after transformation there are "<<output.size()<<" points"<<std::endl;
}


// overloaded function to apply translation and rotation without scaling to PointCloud using quaternion 
void CloudFilter::transformCloud(PointCloud &input, PointCloud &output, Eigen::Quaterniond rotation, Eigen::Vector3d translation){

  PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy of the training cloud
  pcl::copyPointCloud(input,*cloud);

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  // set the translation and rotation of the transformation 
  transform.translation() << translation[0], translation[1], translation[2];
  transform.rotate(rotation);

  // Print the transformation
  //std::cout << transform_2.matrix() << std::endl;

  // Execute the transformation on working copy 
  pcl::transformPointCloud (*cloud, *cloud, transform); 
  // copy to the output cloud
  pcl::copyPointCloud(*cloud, output);
  
  std::cout<<"after transformation there are "<<output.size()<<" points"<<std::endl; 
}

/*
// function to apply moving least squared smoothing to pointcloud
void CloudFilter::smoothCloud(PointCloud &input, PointCloudNormal &output){

  PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy for this function 
  pcl::copyPointCloud(input,*cloud);

  // Create a KD-Tree
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  //pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points; 
  // modify the function output pointcloud directly instead
  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<PointT, pcl::PointXYZRGBNormal> mls;

  // Set parameters
  mls.setComputeNormals (true);
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (output);
  std::cout<<"after smoothing there are "<<output.size()<<"points in the cloud"<<std::endl;
}*/



// templated function for PCL moving least squares smoothing, normal data generated during this process
template <typename point_t, typename point_normal_t> 
void CloudFilter::smoothCloudT(pcl::PointCloud<point_t> &input, pcl::PointCloud<point_normal_t> &output){
  
  //allocate memory and make copy to use as the working copy for this function 
  typename pcl::PointCloud<point_t>::Ptr cloud (new pcl::PointCloud<point_t>); 
  pcl::copyPointCloud(input,*cloud);
  
  std::cout<<"before smoothing there are "<<cloud->size()<<"points in the cloud"<<std::endl;
  // Create a KD-Tree
  typename pcl::search::KdTree<point_t>::Ptr tree (new pcl::search::KdTree<point_t>);

  pcl::MovingLeastSquares<point_t, point_normal_t> mls;

  // Set parameters
  mls.setComputeNormals (true);
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (output);
  std::cout<<"after smoothing there are "<<output.size()<<"points in the cloud"<<std::endl;
}
 
template void CloudFilter::smoothCloudT< pcl::PointXYZRGB, pcl::PointXYZRGBNormal >
              (pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGBNormal> &output);

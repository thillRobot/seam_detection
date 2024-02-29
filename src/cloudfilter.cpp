/*

 filter_cloud applies a series of filters and processes to the input pointcloud
 Tristan Hill - 02/28/2023
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/

#include <string>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//#include "ros/package.h"
//#include <ros/ros.h>

#include "cloudfilter.h"

// DEFINITIONS


// default constructor
CloudFilter :: CloudFilter() {}



//  : config("cloudfilter"), node(ros::NodeHandle n)

CloudFilter :: CloudFilter(const std::string cfg)
{}
//  config=cfg;
//  node=ros::NodeHandle n;
//}


/*
void CloudFilter :: setConfig(const std::string cfg) { // ROS::Rate rate(5) is in intializer list
  
  config=cfg;

}
*/

/*
// function to load the config file(yaml) to pick the data files and set parameters 
int CloudFilter :: loadConfig(void) {

  std::cout<<"|---------- FilterCloud::loadConfig - loading configuration file ---------|"<<std::endl;

  std::cout<<"config file: "<<config<<std::endl;

  // get parameters from rosparam
  node.getParam("input_file", input_file);    
  node.getParam("output_file", output_file);

  std::string s; 
 
  return 0;
}

std::string CloudFilter :: getConfig(void){

  return config;

}
*/

/*

 cloudfilter - applies a series of filters and processes to the input pointcloud
 Tristan Hill - 02/29/2024
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/


#include "cloudfilter.h"
#include <string>
#include <ros/ros.h>
#include <ros/package.h>


// DEFINITIONS


// default constructor

CloudFilter::CloudFilter() 
  : config("cloudfilter")
{}

CloudFilter::CloudFilter(std::string cfg="cloudfilter") 
  : config(cfg)
{}

CloudFilter::~CloudFilter()
{}


void CloudFilter::loadConfig(std::string cfg){

  std::string input_file, output_file;

  config=cfg;
  node.getParam("input_file", input_file);    
  node.getParam("output_file", output_file);

} 

std::string CloudFilter::getConfig(void){

  return config;

} 

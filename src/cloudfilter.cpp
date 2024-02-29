/*

 cloudfilter - applies a series of filters and processes to the input pointcloud
 Tristan Hill - 02/29/2024
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/

#include <string>
#include "cloudfilter.h"

// DEFINITIONS


// default constructor
CloudFilter::CloudFilter() {}

void CloudFilter::loadConfig(std::string cfg){

  config=cfg;

} 

std::string CloudFilter::getConfig(void){

  return config;

} 

#include "cloudfilter.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"cloudfilter_test");
  
  CloudFilter filter;   

  // simple test of calling public members
  std::cout<<"default config: "<<filter.getConfig()<<std::endl; 
  filter.loadConfig("cloudfilter_test");
  std::cout<<"loading config: "<<filter.getConfig()<<std::endl;
 
  std::cout<<"using bounding box: "<<std::endl
           <<"X["<<filter.getBoundingBox()[0]<<","<<filter.getBoundingBox()[1]<<"]"<<std::endl
           <<"Y["<<filter.getBoundingBox()[2]<<","<<filter.getBoundingBox()[3]<<"]"<<std::endl
           <<"Z["<<filter.getBoundingBox()[4]<<","<<filter.getBoundingBox()[5]<<"]"<<std::endl;

  return 0;

}

#include "cloudfilter.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"cloudfilter_test");
  
  CloudFilter filter;   

  std::cout<<"default config: "<<filter.getConfig()<<std::endl;
  
  filter.loadConfig("cloudfilter_test");

  std::cout<<"loading config: "<<filter.getConfig()<<std::endl;
 
  return 0;

}

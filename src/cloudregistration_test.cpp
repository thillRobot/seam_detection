#include "cloudregistration.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"cloudregistration_test");
  
  CloudRegistration registration;   

  // simple test of calling public members
  std::cout<<"default config: "<<registration.getConfig()<<std::endl; 
  registration.loadConfig("cloudregistration_test");
  std::cout<<"loading config: "<<registration.getConfig()<<std::endl;

  return 0;

}

#include "cloudutils.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"cloudutils_test");
  
  CloudUtils utl;   

  // simple test of calling public members
  std::cout<<"default config: "<<utl.getConfig()<<std::endl; 
  utl.loadConfig("test/cloudutils_test");
  std::cout<<"loading config: "<<utl.getConfig()<<std::endl;
 
  return 0;

}

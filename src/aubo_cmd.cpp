/*
aubo_cmd.cpp - example of sending cmd to aubo robot

Tristan Hill - Weld Seam Detection - Tennessee Technological University
Robotics Research Group - Mechanical Engineering

v1.0 - 2022-12-20 

see README.md or https://github.com/thillRobot/seam_detection for documentation
*/
  
// basic file operations
#include <iostream>
#include <fstream>
#include <string> 
#include <sstream>   
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>


#include "ros/package.h"
#include <ros/ros.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "ros/package.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;

bool filter_source_complete=0;
bool registration_complete=0;

// points from teach pose in meters
Eigen::Vector3d P0, P0_inches,   
                P1, P1_inches,
                P2, P2_inches;


int main(int argc, char** argv)
{

  ros::init(argc,argv,"aubo_cmd");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  // setup subcribers for filter_cloud_state
  //ros::Subscriber filter_source_state_sub = node.subscribe("/filter_source/filter_source_state", 1000, filter_source_state_callback);
  
  //ros::Subscriber teach_pose = node.subscribe("/teach_points/teach_points_poses", 1000, teach_points_poses_callback);

  std::stringstream gcode; 

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                    aubo_cmd v1.9                                   "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  
  double intm=0.0254;
  double mtin=1/0.0254;

  P0<<0.241742, -0.657364, 0.0447408;
  P0_inches=P0*mtin;

  //P0_inches << 0, -28, 6;
  //P1_inches << 2, -20, 10;
  //P2_inches << 4, -20, 10;


  //P0=P0_inches*intm;
  P1=P1_inches*intm;
  P2=P2_inches*intm;



  std::cout<<"P0: ["<<P0.x()<<","<<P0.y()<<","<<P0.z()<<"]"<<std::endl;
  std::cout<<"P1: ["<<P1.x()<<","<<P1.y()<<","<<P1.z()<<"]"<<std::endl;
  std::cout<<"P2: ["<<P2.x()<<","<<P2.y()<<","<<P2.z()<<"]"<<std::endl;
  
  std::cout<<"P0_inches: ["<<P0_inches.x()<<","<<P0_inches.y()<<","<<P0_inches.z()<<"]"<<std::endl;
  std::cout<<"P1_inches: ["<<P1_inches.x()<<","<<P1_inches.y()<<","<<P1_inches.z()<<"]"<<std::endl;
  std::cout<<"P2_inches: ["<<P2_inches.x()<<","<<P2_inches.y()<<","<<P2_inches.z()<<"]"<<std::endl;

  
  // write the points to a gcode file called 'move_P1_P2', the source points will be used


  //std::stringstream gcode;
  gcode.str(""); // clear the buffer
  gcode <<"G1 X"<<P0_inches.x()<<" Y"<<P0_inches.y()<<" Z"<<P0_inches.z()<<" A0 B0 C-145 F200";
  //gcode <<"G4 P0.2";
  //gcode <<"G1 X"<<P1_inches.x()<<" Y"<<P1_inches.y()<<" Z"<<P1_inches.z()<<" A60 B10 C175 F150";
  //gcode <<"G4 P0.2";
  //gcode <<"G1 X"<<P2_inches.x()<<" Y"<<P2_inches.y()<<" Z"<<P2_inches.z()<<" A60 B10 C175 F150";


  ros::Publisher gcode_pub = node.advertise<std_msgs::String> ("/motion/move_cmd", 1,true);
  std_msgs::String gcode_msg;


  gcode_msg.data=gcode.str();
  gcode_pub.publish(gcode_msg);

  //publish forever
  while(ros::ok())
  {
      
    ros::spinOnce();
    loop_rate.sleep();
  
  }

  return 0;
}

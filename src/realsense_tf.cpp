/*

  node to setup tf for SEAM_DETECTION and ROBOT_VISION/INTELREALSENSE rgbd camera

  this replaces the common node 'setup_tf'
  migrated from TF to TF2 and back again to old TF way just use RPY... (this seems a bit backwards)

  Node: realsense_tf.cpp
  Package: seam_detection
  Tristan Hill - 02/11/2024 

*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <math.h>


bool playback=true;

int main(int argc, char** argv){


    std::cout<<"*************************************************************"<<std::endl;
    std::cout<<"***************** realsense TF v1.0 *************************"<<std::endl;
    std::cout<<"*************************************************************"<<std::endl;

    ros::init(argc, argv, "realsense_tf");
    ros::NodeHandle n;

    ros::Rate r(10);

    // get boolen parameters 
    
    // node.getParam("playback", playback);

    tf::TransformBroadcaster broadcaster;
    float d2r=M_PI/180.0;
    float in2m=0.0254;
    while(n.ok())
    {

      broadcaster.sendTransform( // take the conversion from rpy to quaternion out of the loop, do this later 
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"map","base_link"));

//    broadcaster.sendTransform(
//      tf::StampedTransform(
//      tf::Transform(tf::createQuaternionFromRPY(0,M_PI/2,0), tf::Vector3(0.0, 0.0, 32.0*in2m)),
//      ros::Time::now(),"base_link","camera_link"));

     broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 65.25*in2m)),
        ros::Time::now(),"base_link","tripod_link"));
    
     broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(M_PI+30*d2r,0,0), tf::Vector3(0.0, 5.75*in2m, 0.0)),
        ros::Time::now(),"tripod_link","camera_link"));
      
      if (playback){
        broadcaster.sendTransform(
          tf::StampedTransform(
          tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0)),
          ros::Time::now(),"camera_link","camera_color_optical_frame"));
      }
     
      r.sleep();
      ros::spinOnce();
    }

    ros::spin();
}


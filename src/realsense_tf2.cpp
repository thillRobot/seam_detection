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

    ros::Rate r(50);

    tf::TransformBroadcaster broadcaster;
    //static tf2_ros::StaticTransformBroadcaster broadcaster; // the tf2 way
    // make a 'zero' transform from 'base_link' to 'map' - this is a 'geometry msg'   // zero translation// zero rotation
    
    //geometry_msgs::TransformStamped *T_base_map (new geometry_msgs::TransformStamped); // from 'base_link' to 'map'
    //geometry_msgs::TransformStamped *T_camera_base (new geometry_msgs::TransformStamped); // from 'camera_link' to 'base_link'
    
    //T_base_map->header.stamp = ros::Time::now();T_base_map->header.frame_id = "map";T_base_map->child_frame_id ="base_link";
    //T_base_map->transform.translation.x = 0; T_base_map->transform.translation.y = 0; T_base_map->transform.translation.z = 0;
    //T_base_map->transform.rotation.x = 0; T_base_map->transform.rotation.y = 0; T_base_map->transform.rotation.z = 0;T_base_map->transform.rotation.w = 1;

    //T_camera_base->header.stamp = ros::Time::now();T_camera_base->header.frame_id = "base_link";T_camera_base->child_frame_id ="camera_link";
    //T_camera_base->transform.translation.x = 0; T_camera_base->transform.translation.y = 0; T_camera_base->transform.translation.z = 1.0;
    //T_camera_base->transform.rotation.x = 0; T_camera_base->transform.rotation.y = 0; T_camera_base->transform.rotation.z = 0;T_camera_base->transform.rotation.w = 1;

    //ros::spin();
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
        tf::Transform(tf::createQuaternionFromRPY(M_PI+10*d2r,0,0), tf::Vector3(0.0, 5.75*in2m, 0.0)),
        ros::Time::now(),"tripod_link","camera_link"));
      
      if (playback){
        broadcaster.sendTransform(
          tf::StampedTransform(
          tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0)),
          ros::Time::now(),"camera_link","camera_color_optical_frame"));
      }
      //T_base_map->header.stamp = ros::Time::now();  // from the tf2 way
      //broadcaster.sendTransform(*T_base_map);

      //T_camera_base->header.stamp = ros::Time::now();
      //broadcaster.sendTransform(*T_camera_base);

      r.sleep();
      ros::spinOnce();
    }

    ros::spin();
}


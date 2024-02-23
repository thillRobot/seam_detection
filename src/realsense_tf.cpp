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
    std::cout<<"***************** realsense TF v1.9 *************************"<<std::endl;
    std::cout<<"*************************************************************"<<std::endl;

    ros::init(argc, argv, "realsense_tf");
    ros::NodeHandle node;
    
    float d2r=M_PI/180.0;
    float in2m=0.0254;
      
    float camera_roll, camera_yaw, camera_pitch,  
	  camera_x, camera_y, camera_z, 
          camera_mount_x, camera_mount_y, camera_mount_z,
          camera_mount_roll, camera_mount_yaw, camera_mount_pitch,   
          tripod_roll, tripod_yaw, tripod_pitch,
          tripod_x, tripod_y, tripod_z; 
 
    node.getParam("camera_roll", camera_roll);
    node.getParam("camera_pitch", camera_pitch);
    node.getParam("camera_yaw", camera_yaw);
    
    camera_roll=camera_roll*d2r;  //+M_PI;
    camera_pitch=camera_pitch*d2r;
    camera_yaw=camera_yaw*d2r;
    
    node.getParam("camera_x", camera_x);
    node.getParam("camera_y", camera_y);
    node.getParam("camera_z", camera_z);

    camera_x=camera_x*in2m;
    camera_y=camera_y*in2m;
    camera_z=camera_z*in2m;

    std::cout<<"camera_roll: "<<camera_roll
	     <<", camera_pitch: "<<camera_pitch
	     <<", camera_yaw: "<<camera_yaw<<std::endl;	

    node.getParam("tripod_roll", tripod_roll);
    node.getParam("tripod_pitch", tripod_pitch);
    node.getParam("tripod_yaw", tripod_yaw);
    
    tripod_roll=tripod_roll*d2r;
    tripod_pitch=tripod_pitch*d2r;
    tripod_yaw=tripod_yaw*d2r;
    
    node.getParam("tripod_x", tripod_x);
    node.getParam("tripod_y", tripod_y);
    node.getParam("tripod_z", tripod_z);

    tripod_x=tripod_x*in2m;
    tripod_y=tripod_y*in2m;
    tripod_z=tripod_z*in2m;


    node.getParam("camera_mount_roll", camera_mount_roll);
    node.getParam("camera_mount_pitch", camera_mount_pitch);
    node.getParam("camera_mount_yaw", camera_mount_yaw);
    
    camera_mount_roll=camera_mount_roll*d2r;
    camera_mount_pitch=camera_mount_pitch*d2r;
    camera_mount_yaw=camera_mount_yaw*d2r;
    
    node.getParam("camera_mount_x", camera_mount_x);
    node.getParam("camera_mount_y", camera_mount_y);
    node.getParam("camera_mount_z", camera_mount_z);

    camera_mount_x=camera_mount_x*in2m;
    camera_mount_y=camera_mount_y*in2m;
    camera_mount_z=camera_mount_z*in2m;
    ros::Rate r(10);

    // get boolen parameters 
    
    // node.getParam("playback", playback);

    tf::TransformBroadcaster broadcaster;
    while(node.ok())
    {
      // take the conversion from rpy to quaternion out of the loop, do this later 
      broadcaster.sendTransform( 
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"map","base_link"));

      broadcaster.sendTransform( 
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"world","map"));

//    broadcaster.sendTransform(
//      tf::StampedTransform(
//      tf::Transform(tf::createQuaternionFromRPY(0,M_PI/2,0), tf::Vector3(0.0, 0.0, 32.0*in2m)),
//      ros::Time::now(),"base_link","camera_link"));

//     broadcaster.sendTransform(
//        tf::StampedTransform(
//        tf::Transform(tf::createQuaternionFromRPY(tripod_roll, tripod_pitch, tripod_yaw), tf::Vector3(tripod_x, tripod_y, tripod_z)),
//        ros::Time::now(),"base_link","tripod_link"));
    
 //    broadcaster.sendTransform(
 //       tf::StampedTransform(
 //       tf::Transform(tf::createQuaternionFromRPY(camera_roll,camera_pitch,camera_yaw), tf::Vector3(camera_x, camera_y, camera_z)),
 //       ros::Time::now(),"tripod_link","camera_link"));
      
     broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(camera_mount_roll,camera_mount_pitch,camera_mount_yaw), 
		      tf::Vector3(camera_mount_x, camera_mount_y, camera_mount_z)),
        	      ros::Time::now(),"T6","camera_mount"));
    
     broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(camera_roll,camera_pitch,camera_yaw), 
	              tf::Vector3(camera_x, camera_y, camera_z)),
                      ros::Time::now(),"camera_mount","camera_link"));
              
     playback=0;         
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


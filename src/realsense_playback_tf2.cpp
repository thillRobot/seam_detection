/*

  node to setup tf for SEAM_DETECTION and ROBOT_VISION/INTELREALSENSE rgbd camera

  this replaces the common node 'setup_tf'
  migrated from TF to TF2 to solve time extrapolation, TF_REPEATED_DATA issue

  Node: realsense_playback_tf.cpp
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

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv){

    std::cout<<"*************************************************************"<<std::endl;
    std::cout<<"***************** realsense playback TF(2) v1.0 ****************"<<std::endl;
    std::cout<<"*************************************************************"<<std::endl;

    ros::init(argc, argv, "realsense_playback_tf2");
    ros::NodeHandle node;

    ros::Rate rate(10);
   
    // from 'camera_color_optical_link' to 'camera_link'
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped *T_optical_camera (new geometry_msgs::TransformStamped);

    T_optical_camera->header.stamp = ros::Time::now();
    T_optical_camera->header.frame_id = "camera_link";
    T_optical_camera->child_frame_id ="camera_color_optical_frame";

    tf2::Quaternion quat_tf;
    double roll, pitch, yaw;
    
    roll=0.0; pitch=-M_PI; yaw=0.0;
    quat_tf.setRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf);
    
    T_optical_camera->transform.translation.x = 0.0; 
    T_optical_camera->transform.translation.y = 0.0; 
    T_optical_camera->transform.translation.z = 0.0;
    //T_optical_camera->transform.rotation.x = 0.0;
    //T_optical_camera->transform.rotation.y = 0.0;
    //T_optical_camera->transform.rotation.z = 0.0;
    //T_optical_camera->transform.rotation.w = 1.0;
    T_optical_camera->transform.rotation=quat_msg;


    while(node.ok()){
  
      T_optical_camera->header.stamp = ros::Time::now();
      broadcaster.sendTransform(*T_optical_camera);

      rate.sleep();
      ros::spinOnce(); 
    }

    ros::spin();
}


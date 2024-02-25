/*

  node to setup tf for SEAM_DETECTION and ROBOT_VISION/INTELREALSENSE rgbd camera with dynamic reconfigure

  this replaces the common node 'setup_tf' and allows the TFs to be adjusted during runtime
  
  Node: reconfigure_tf.cpp
  Package: seam_detection
  Tristan Hill - 02/25/2024

*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <math.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <seam_detection/ReconfigureTFConfig.h>

bool playback=true;

//function to set dynamic_reconfigure double parameter value
void reconfigureDouble(std::string name, double value){
   
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter param;    
    dynamic_reconfigure::Config conf;
    
    param.name=name;
    param.value=value;
    conf.doubles.push_back(param);   
   
    srv_req.config = conf;

    std::cout<<"name: "<< name << " value: "<< value <<std::endl;

    if (ros::service::call("/reconfigure_server/set_parameters", srv_req, srv_resp)) {
      ROS_INFO("call to set reconfigure_server parameters succeeded");
    } else {
      ROS_INFO("call to set reconfigure_server parameters failed");
    } 

}


int main(int argc, char** argv){

    std::cout<<"*************************************************************"<<std::endl;
    std::cout<<"************************ reconfigure_tf v1.9 ****************"<<std::endl;
    std::cout<<"*************************************************************"<<std::endl;

    ros::init(argc, argv, "reconfigure_tf");
    ros::NodeHandle node; 
    ros::Rate rate(10);

    double d2r=M_PI/180.0;
    double in2m=0.0254;
      
    double camera_roll, camera_yaw, camera_pitch,  
	        camera_x, camera_y, camera_z, 
          camera_mount_x, camera_mount_y, camera_mount_z,
          camera_mount_roll, camera_mount_yaw, camera_mount_pitch,   
          tripod_roll, tripod_yaw, tripod_pitch,
          tripod_x, tripod_y, tripod_z; 
 
    // get parameter values from rosparam
    node.getParam("camera_roll", camera_roll);
    node.getParam("camera_pitch", camera_pitch);
    node.getParam("camera_yaw", camera_yaw);
    
    node.getParam("camera_x", camera_x);
    node.getParam("camera_y", camera_y);
    node.getParam("camera_z", camera_z);
    
    node.getParam("tripod_roll", tripod_roll);
    node.getParam("tripod_pitch", tripod_pitch);
    node.getParam("tripod_yaw", tripod_yaw);
    
    node.getParam("tripod_x", tripod_x);
    node.getParam("tripod_y", tripod_y);
    node.getParam("tripod_z", tripod_z);
    
    node.getParam("camera_mount_roll", camera_mount_roll);
    node.getParam("camera_mount_pitch", camera_mount_pitch);
    node.getParam("camera_mount_yaw", camera_mount_yaw);
    
    node.getParam("camera_mount_x", camera_mount_x);
    node.getParam("camera_mount_y", camera_mount_y);
    node.getParam("camera_mount_z", camera_mount_z);
   
    // set dynamic reconfigure parameters from rosparam values
    reconfigureDouble("camera_roll", camera_roll);
    reconfigureDouble("camera_pitch",camera_pitch);
    reconfigureDouble("camera_yaw",camera_yaw);
    reconfigureDouble("camera_x",camera_x);
    reconfigureDouble("camera_y",camera_y);
    reconfigureDouble("camera_z",camera_z);
    
    tf::TransformBroadcaster broadcaster;
    while(node.ok())
    {

      // get updated dynamic reconfigure parameters from rosparam to be used in transforms
      node.getParam("/reconfigure_server/camera_roll", camera_roll);
      node.getParam("/reconfigure_server/camera_pitch", camera_pitch);
      node.getParam("/reconfigure_server/camera_yaw", camera_yaw);
      
      node.getParam("/reconfigure_server/camera_x", camera_x);
      node.getParam("/reconfigure_server/camera_y", camera_y);
      node.getParam("/reconfigure_server/camera_z", camera_z);
      
      node.getParam("/reconfigure_server/tripod_roll", tripod_roll);
      node.getParam("/reconfigure_server/tripod_pitch", tripod_pitch);
      node.getParam("/reconfigure_server/tripod_yaw", tripod_yaw);
      
      node.getParam("/reconfigure_server/tripod_x", tripod_x);
      node.getParam("/reconfigure_server/tripod_y", tripod_y);
      node.getParam("/reconfigure_server/tripod_z", tripod_z);
      
      node.getParam("/reconfigure_server/camera_mount_roll", camera_mount_roll);
      node.getParam("/reconfigure_server/camera_mount_pitch", camera_mount_pitch);
      node.getParam("/reconfigure_server/camera_mount_yaw", camera_mount_yaw);
      
      node.getParam("/reconfigure_server/camera_mount_x", camera_mount_x);
      node.getParam("/reconfigure_server/camera_mount_y", camera_mount_y);
      node.getParam("/reconfigure_server/camera_mount_z", camera_mount_z);
         
      // convert units (deg->rad), (in->m) 
      camera_roll=camera_roll*d2r;  //+M_PI;
      camera_pitch=camera_pitch*d2r;
      camera_yaw=camera_yaw*d2r;
      
      camera_x=camera_x*in2m;
      camera_y=camera_y*in2m;
      camera_z=camera_z*in2m;

      tripod_roll=tripod_roll*d2r;
      tripod_pitch=tripod_pitch*d2r;
      tripod_yaw=tripod_yaw*d2r;
      
      tripod_x=tripod_x*in2m;
      tripod_y=tripod_y*in2m;
      tripod_z=tripod_z*in2m;

      camera_mount_roll=camera_mount_roll*d2r;
      camera_mount_pitch=camera_mount_pitch*d2r;
      camera_mount_yaw=camera_mount_yaw*d2r;
      
      camera_mount_x=camera_mount_x*in2m;
      camera_mount_y=camera_mount_y*in2m;
      camera_mount_z=camera_mount_z*in2m;
      
      // broadcast transformations using updated parameter values
      broadcaster.sendTransform( 
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"map","base_link"));

      broadcaster.sendTransform( 
        tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"world","map"));

//     broadcaster.sendTransform(
//        tf::StampedTransform(
//        tf::Transform(tf::createQuaternionFromRPY(tripod_roll, tripod_pitch, tripod_yaw), 
//                      tf::Vector3(tripod_x, tripod_y, tripod_z)),
//        ros::Time::now(),"base_link","tripod_link"));
    
//     broadcaster.sendTransform(
//        tf::StampedTransform(
//        tf::Transform(tf::createQuaternionFromRPY(camera_roll,camera_pitch,camera_yaw), 
//                      tf::Vector3(camera_x, camera_y, camera_z)),
//        ros::Time::now(),"tripod_link","camera_link"));
      
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
     
      rate.sleep();
      ros::spinOnce();
    }

    ros::spin();
}


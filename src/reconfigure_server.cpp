/*

   server node for dynamic reconfigure
   note: service call to update reconfigure params must be made from separate node

   Node: reconfigure_server.cpp
   Package: seam_detection
   Tristan Hill - 02/25/2024

*/

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <seam_detection/ReconfigureTFConfig.h>


std::vector<double> camera_angle, camera_position, camera_mount_angle, camera_mount_position;

void callback(seam_detection::ReconfigureTFConfig &config, uint32_t level) {
 
   ROS_INFO("Updated Camera TF: %f %f %f %f %f %f",
             config.camera_roll,
             config.camera_pitch,
             config.camera_yaw,
             config.camera_x,
             config.camera_y,
             config.camera_z);
   ROS_INFO("Updated Camera Mount TF: %f %f %f %f %f %f",
             config.camera_mount_roll,
             config.camera_mount_pitch,
             config.camera_mount_yaw,
             config.camera_mount_x,
             config.camera_mount_y,
             config.camera_mount_z);
 
  camera_angle.push_back(config.camera_pitch);
  camera_angle.push_back(config.camera_yaw);
  camera_angle.push_back(config.camera_roll);
  camera_position.push_back(config.camera_x);
  camera_position.push_back(config.camera_y);
  camera_position.push_back(config.camera_z);
 
  camera_mount_angle.push_back(config.camera_mount_pitch);
  camera_mount_angle.push_back(config.camera_mount_yaw);
  camera_mount_angle.push_back(config.camera_mount_roll);
  camera_mount_position.push_back(config.camera_mount_x);
  camera_mount_position.push_back(config.camera_mount_y);
  camera_mount_position.push_back(config.camera_mount_z);
 
 }

int main(int argc, char **argv) {
  ros::init(argc, argv, "reconfigure_server");
  
  dynamic_reconfigure::Server<seam_detection::ReconfigureTFConfig> server;
  dynamic_reconfigure::Server<seam_detection::ReconfigureTFConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}


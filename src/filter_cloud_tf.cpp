/*
  This is a node to setup tf for SEAM_DETECTION
  this replaces the common node 'setup_tf'
  this node has migrated from TF to TF2 

  Node: seam_detection_tf.cpp
  Package: seam_detection

  Tristan Hill     - 02/23/2020
  updated for v1.5 - 03/10/2021 
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv){


    std::cout<<"*************************************************************"<<std::endl;
    std::cout<<"***************** Filter Cloud TF v1.5 ********************"<<std::endl;
    std::cout<<"*************************************************************"<<std::endl;

    ros::init(argc, argv, "seam_detection_tf");
    ros::NodeHandle n;

    ros::Rate r(20);

    //tf::TransformBroadcaster broadcaster;
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // make a 'zero' transform from 'base_link' to 'map' - this is a 'geometry msg'   // zero translation// zero rotation
    geometry_msgs::TransformStamped *T_base_link_map (new geometry_msgs::TransformStamped); // from 'base_link' to 'map'
    //geometry_msgs::TransformStamped *T_map_world (new geometry_msgs::TransformStamped); // from 'map' to 'world'

    T_base_link_map->header.stamp = ros::Time::now();T_base_link_map->header.frame_id = "map";T_base_link_map->child_frame_id ="base_link";
    T_base_link_map->transform.translation.x = 0; T_base_link_map->transform.translation.y = 0; T_base_link_map->transform.translation.z = 0;
    T_base_link_map->transform.rotation.x = 0; T_base_link_map->transform.rotation.y = 0; T_base_link_map->transform.rotation.z = 0;T_base_link_map->transform.rotation.w = 1;

    //T_map_world->header.stamp = ros::Time::now();T_map_world->header.frame_id = "world";T_map_world->child_frame_id ="map";
    //T_map_world->transform.translation.x = 0; T_map_world->transform.translation.y = 0; T_map_world->transform.translation.z = 0;
    //T_map_world->transform.rotation.x = 0; T_map_world->transform.rotation.y = 0; T_map_world->transform.rotation.z = 0;T_map_world->transform.rotation.w = 1;

    ros::spinOnce();

    while(n.ok())
    {

      T_base_link_map->header.stamp = ros::Time::now();
      static_broadcaster.sendTransform(*T_base_link_map);

      //T_map_world->header.stamp = ros::Time::now();
      //static_broadcaster.sendTransform(*T_map_world);
 
        r.sleep();
        ros::spinOnce();
    }

    ros::spin();
}


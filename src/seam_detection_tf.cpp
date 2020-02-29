/*
  This is a node to setup tf for SEAM_DETECTION
  this replaces the common node 'setup_tf'
  this node has migrated from TF to TF2 

  Node: seam_detection_tf.cpp
  Package: seam_detection

  Tristan Hill, 02/23/2020

*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
//#include "Vector3.h"
//#include "Quaternion.h"
//#include <Matrix3x3.h>

// tf setup does not need a callback, for now.
/*
void dataCallback1(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("CALLBACK!");
    ROS_INFO("I heard: [%f]", msg->data);
    //angle1=-(65-msg->data)*(3.14159/180.0);
    angle1=-(90-msg->data)*(3.14159/180.0);
}
*/
int main(int argc, char** argv){

    ros::init(argc, argv, "seam_detection_tf");
    ros::NodeHandle n;

    ros::Rate r(20);

    //tf::TransformBroadcaster broadcaster;
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // make a 'zero' transform from 'base_link' to 'map' - this is a 'geometry msg'   // zero translation// zero rotation
    geometry_msgs::TransformStamped *T_base_link_map (new geometry_msgs::TransformStamped); // from 'base_link' to 'map'

    T_base_link_map->header.stamp = ros::Time::now();T_base_link_map->header.frame_id = "map";T_base_link_map->child_frame_id ="base_link";

    T_base_link_map->transform.translation.x = 0;
    T_base_link_map->transform.translation.y = 0;
    T_base_link_map->transform.translation.z = 0;

    T_base_link_map->transform.rotation.x = 0;
    T_base_link_map->transform.rotation.y = 0;
    T_base_link_map->transform.rotation.z = 0;
    T_base_link_map->transform.rotation.w = 1;

    ros::spinOnce();

    while(n.ok())
    {
      T_base_link_map->header.stamp = ros::Time::now();
      static_broadcaster.sendTransform(*T_base_link_map);
        //ROS_INFO("Sending Transform with angle %f",angle);
/*
        broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0.0)),
            //tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
            ros::Time::now(),"map","base_link"));
            */
       /*// The 'laser' link is not currently used by 'seam_detection'
       broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::createQuaternionFromRPY(0,angle1,angle2), tf::Vector3(0.0, 0.0, 0.0)),
            //tf::Transform(mat, tf::Vector3(0.0, 0.0, 0.0)),

            //tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.0)),
            ros::Time::now(),"base_link","laser"));
        */
        //ROS_INFO("Transform Sent");
        r.sleep();
        ros::spinOnce();
    }

    ros::spin();
}

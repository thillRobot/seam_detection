#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <tf/transform_datatypes.h>

//#include "Vector3.h"
//#include "Quaternion.h"
//#include <Matrix3x3.h>

float angle1;
float angle2;

void dataCallback1(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("CALLBACK!");
    ROS_INFO("I heard: [%f]", msg->data);
    //angle1=-(65-msg->data)*(3.14159/180.0);
    angle1=-(90-msg->data)*(3.14159/180.0);
}

void dataCallback2(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("CALLBACK!");
    ROS_INFO("I heard: [%f]", msg->data);
    angle2=msg->data*(3.14159/180.0);
}

int main(int argc, char** argv){
 
    ros::init(argc, argv, "setup_tf");
    ros::NodeHandle n;

    ros::Rate r(20);

    tf::TransformBroadcaster broadcaster;

    ros::Subscriber sub1 = n.subscribe("/servo1", 1000, dataCallback1);
    ros::Subscriber sub2 = n.subscribe("/servo2", 1000, dataCallback2);
    
    tf::Matrix3x3 mat;
    mat.setValue(1,0,0,0,1,0,0,0,1);

    ros::spinOnce();    

    angle1=0;
    angle2=0;
    //while(angle<0)
    //{
    //    ROS_INFO("Waiting for Callback.");
    //}

    while(n.ok())
    {
        //ROS_INFO("Sending Transform with angle %f",angle);
        broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0.0, 0.0, 0.0)),
            //tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
            ros::Time::now(),"map","base_link"));

	    broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::createQuaternionFromRPY(0,angle1,angle2), tf::Vector3(0.0, 0.0, 0.0)),            
            //tf::Transform(mat, tf::Vector3(0.0, 0.0, 0.0)),            
                        
            //tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0.0)),
            ros::Time::now(),"base_link","laser"));
	
        //ROS_INFO("Transform Sent");
        r.sleep();
        ros::spinOnce();
    }


    ros::spin();
}

// transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
// transform.setRotation( tf::createQuaternionFromRPY(0,0,M_PI/2) );


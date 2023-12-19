/*
// This program gets a set of teach points from the robot arm 
// to use as reference points for the target scan
// Tristan Hill - 08/28/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <ros/ros.h>
#include "ros/package.h"
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

bool teach_points_state=0;
int idx=0; // index for set of points
int num_points; 

// global parameters for callback access
std::string output_path, output_file; 

geometry_msgs::Pose current_pose;
geometry_msgs::PoseArray teach_poses;

// callback for tool pose simple
void current_pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  current_pose=*msg;
  //source_saved=msg->data;
  //ROS_INFO("current pose: %f", msg->position.x);
}

//callback for teach click
void teach_pose_callback(const std_msgs::Int8::ConstPtr& msg)
{

  //source_saved=msg->data;
  ROS_INFO("teaching point: %i", idx);
  ROS_INFO("current_pose x: %f", current_pose.position.x);
  ROS_INFO("current_pose y: %f", current_pose.position.y);
  ROS_INFO("current_pose z: %f", current_pose.position.z);

  teach_poses.poses.push_back(current_pose);

  idx++;
  if (idx==num_points){
    teach_points_state=1; 
  }

}


int main(int argc, char** argv)
{

  ros::init(argc,argv,"teach_points");
  ros::NodeHandle node;
  ros::Rate loop_rate(5);
  
  ros::Subscriber current_pose_sub = node.subscribe("/aubo_robot/current_pose_tool0_basic",10, current_pose_callback);
  ros::Subscriber teach_pose_sub = node.subscribe("/cr_weld/teach_pose",10, teach_pose_callback);
  
  ros::Publisher teach_points_state_pub = node.advertise<std_msgs::Bool> ("/teach_points/teach_points_state", 1);
  ros::Publisher teach_points_pub = node.advertise<geometry_msgs::PoseArray> ("/teach_points/teach_points_poses", 1);
  ros::Publisher free_drive_pub = node.advertise<std_msgs::Bool> ("/cr_weld/free_drive", 1);

  std_msgs::Bool teach_points_state_msg, free_drive_msg;
  geometry_msgs::PoseArray teach_points_msg;

  bool free_drive;


  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points v1.x                              "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points: loading configuration file       "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // no, this is not true, the config file is read in roslaunch, not by the node
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // get parameters from config file
  node.getParam("teach_points/num_points", num_points);

  std::cout<<"teach_points: preparing to teach "<<num_points<<" num_points"<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points: setup complete                   "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;


  if (teach_points_state){
    std::cout<<"teach_points: poses taught "<<std::endl;
    for(int i=0; i<num_points; i++){
      std::cout<<"pose"<<i<<": ["<<teach_poses.poses[i].position.x<<","<<teach_poses.poses[i].position.y<<","<<teach_poses.poses[i].position.z<<"]"<<std::endl;
    }
  }

  //publish forever
  while(ros::ok())
  {

    if (!teach_points_state){
      free_drive=1; // turn on free drive until teaching points is complete
    }else{
      free_drive=0;
    }
    
    free_drive_msg.data=free_drive;
    free_drive_pub.publish(free_drive_msg);

    teach_points_state_msg.data=teach_points_state;
    teach_points_state_pub.publish(teach_points_state_msg);

    teach_points_msg=teach_poses;
    teach_points_pub.publish(teach_points_msg);

    //source_saved_msg.data=cloud_saved;
    //source_saved_pub.publish(source_saved_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     teach_points: complete                         "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;
  return 0;
}



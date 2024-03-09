// This program applies loads a pointcloud from file and applies a rotation
// and a translation is possible if needed.  
// A .pcd file is read as input and a new .pcd file is written as output
// Tristan Hill - 08/22/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation

#include "ros/package.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;

bool get_cloud_complete=0;
bool filtered_cloud_saved=0;

void get_cloud_stateCallback(const std_msgs::Bool::ConstPtr& msg)
{
  //ROS_INFO("I heard scan_state: [%d]", msg->data);
  if (!msg->data){
    ROS_INFO("get_cloud in progress, waiting to begin filtering ...");
  }
  else if (msg->data&&!get_cloud_complete){
    ROS_INFO("get_cloud complete, beginning filtering");
    get_cloud_complete=1;
  }
}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"rotate_cloud");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);
  
  std_msgs::Bool filter_cloud_state_msg;
  filter_cloud_state_msg.data=filtered_cloud_saved;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     rotate_cloud v1.8                              "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     rotate_cloud: loading configuration file       "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters 
  bool save_output;
  node.getParam("rotate_cloud/save_output", save_output);

  // parameters that contain strings  
  std::string input_path, output_path, target_path, input_file, output_file, target_file; 
  node.getParam("rotate_cloud/input_file", input_file);
  input_path=packagepath+'/'+input_file;
  node.getParam("rotate_cloud/output_file", output_file);
  output_path=packagepath+'/'+output_file;

  std::cout<<"Debug0"<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_cloud: preparing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  
  // instantiate some cloud pointers
  PointCloud::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZ>);

  std::cout << "Loading cloud files:" <<std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, *cloud_input) == -1)
  {
      std::cout<<"Couldn't read cloud_input file:"<<input_path;
      return (-1);
  }
  std::cout << "Loaded input cloud file: "<< input_path <<std::endl<<
    cloud_input->width * cloud_input->height << " Data points from "<< input_path << std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                    rotate_cloud: processing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // Rotate pointcloud by alpha, beta, gamma about z,y,x
  // set rotation and origin of a quaternion for the tf transform object
  double alpha, beta, gamma, dtr;
  dtr=M_PI/180.0;

  // rotation angles for yaw pitch roll
  alpha=0*dtr;beta=0*dtr;gamma=90*dtr; 

  // rotation matrix for Yaw Pitch Roll by alpha gamma beta
  tf::Matrix3x3 R_zyx(cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma),
                       sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma),
                       -sin(beta)          , cos(beta)*sin(gamma)                                 , cos(beta)*cos(gamma));  

  // quaternion for previous rotation matrix
  tf::Quaternion q_zyx;
  R_zyx.getRotation(q_zyx); // sets quaternion q_intr with rotation from R_intr (returns normalized quaternion?, check on this)

  tf::StampedTransform *T_zyx (new tf::StampedTransform); 
  T_zyx->setRotation(q_zyx);
  T_zyx->setOrigin(tf::Vector3(0, 0, 0)); // no translation component of the transformation (is 0,0,0 default?)
  // need to normalize quaternion here?

  // transform source cloud to ith intermediate starting position 
  pcl_ros::transformPointCloud(*cloud_input, *cloud_rotated, *T_zyx);

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                    rotate_cloud: saving pointcloud data            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // save rotated cloud 
  if(save_output){
    pcl::io::savePCDFileASCII (output_path, *cloud_rotated);
    std::cout<<"Rotated cloud written to:"<< output_path <<std::endl;
  }

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                   rotate_cloud: pointcloud processing complete     "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                   rotate_cloud: preparing visualization            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  ros::Publisher pub_input = node.advertise<PointCloud> ("/cloud_input", 1) ;
  ros::Publisher pub_rotated = node.advertise<PointCloud> ("/cloud_rotated", 1) ;

  cloud_input->header.frame_id = "base_link";
  cloud_rotated->header.frame_id = "base_link";

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                        rotate_cloud: complete                      "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  //publish forever
  while(ros::ok())
  {

      pub_input.publish(cloud_input);
      pub_rotated.publish(cloud_rotated);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;

}
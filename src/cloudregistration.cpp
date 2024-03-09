/*

 cloudregistration - registration functions for finding transformation between pointsclouds
 Tristan Hill - 03/04/2024
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/

#include "cloudregistration.h"
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
//#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <geometry_msgs/TransformStamped.h>
#include <pcl/registration/icp.h>

// DEFINITIONS

// default constructor

CloudRegistration::CloudRegistration() 
  : config("cloudregistration"){
  
  // find the path to the this package (seam_detection)
  package_path = ros::package::getPath("seam_detection");

}


CloudRegistration::CloudRegistration(std::string cfg="cloudregistration", int idx=0) 
  : config(cfg){
}

CloudRegistration::~CloudRegistration(){
}


void CloudRegistration::loadConfig(std::string cfg){

  config=cfg;
  
  std::cout<<"|---------- CloudRegistration::loadConfig() ------------|"<<std::endl; 
 
  node.getParam("input_file", input_file);    
  node.getParam("output_file", output_file);
 
  node.getParam("icp_max_corr_dist", icp_max_corr_dist);    
  node.getParam("icp_max_iter", icp_max_iter);
  node.getParam("icp_trns_epsl", icp_trns_epsl);    
  node.getParam("icp_ecld_fitn_epsl", icp_ecld_fitn_epsl);   
  node.getParam("icp_ran_rej_thrsh", icp_ran_rej_thrsh);

} 

std::string CloudRegistration::getConfig(void){

  return config;

}


// function REGISTER_CLOUD_ICP finds the transform between two pointclouds using PCL::IterativeClosestPoint

template <typename point_t>
double CloudRegistration::registerCloudICP( pcl::PointCloud<point_t> &source, pcl::PointCloud<point_t> &target, 
                                              tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
                                              geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA){

  std::cout<<"|---------- CloudRegistration::registerCloudICP ----------|"<<std::endl;  
  
  //loadConfig(config);  

  // get size of inputs clouds
  int Ns = source.size();
  int Nt = target.size();
  std::cout<<"BEGINNING ICP REGISTRATION" << std::endl;
  std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;
  
  std::cout<<"Using Search Parameters:"<< std::endl;
  std::cout<<"Max Correspondence Distance = "<< icp_max_corr_dist <<std::endl;
  std::cout<<"Maximum Number of Iterations = "<< icp_max_iter <<std::endl;
  std::cout<<"Transformation Epsilon = "<< icp_trns_epsl <<std::endl;
  std::cout<<"Euclidean Distance Difference Epsilon = "<< icp_ecld_fitn_epsl <<std::endl;
  std::cout<<"RANASC Rejection Threshold = "<< icp_ran_rej_thrsh <<std::endl;

  // perform ICP on the lidar and cad clouds
  pcl::IterativeClosestPoint<point_t, point_t> icp;
  pcl::PointCloud<point_t> Final;
  
  Eigen::MatrixXf T_result, T_inverse;
  
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (icp_max_corr_dist);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (icp_max_iter);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (icp_trns_epsl);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (icp_ecld_fitn_epsl);
  // Set the RANSAC Outlier Rejection Threshold
  icp.setRANSACOutlierRejectionThreshold (icp_ran_rej_thrsh);
  
  // these copies seem like a waste to me, figure out how to cut these out
  // make a copy of the LiDAR(source) cloud  
  //working copy of the source cloud
  typename pcl::PointCloud<point_t>::Ptr src (new pcl::PointCloud<point_t>);      
  pcl::copyPointCloud(source,*src);
  // make a copy of the CAD(target) cloud 
  //working copy of the target cloud
  typename pcl::PointCloud<point_t>::Ptr tgt (new pcl::PointCloud<point_t>);       
  pcl::copyPointCloud(target,*tgt);
  
  icp.setInputSource(src); // source (moved during ICP) cloud
  icp.setInputTarget(tgt); // target (fixed) cloud
  icp.align(Final); // perform ICP registration
  
  T_result=icp.getFinalTransformation(); // get the resutls of ICP
  T_inverse=T_result.inverse();          // also store the inverse transformation
  
  double fit_score;
  fit_score=icp.getFitnessScore(); // record fitness score
  //std::cout << "ICP completed with fitness score: " << fit_score << std::endl;
  //std::cout << "ICP COMPLETED" << std::endl;
  //std::cout << "max iterations:" << icp.getMaximumIterations() << std::endl;
  std::cout << "ICP has converged:" << icp.hasConverged() << ", score: " << icp.getFitnessScore() << std::endl;
  std::cout << "transformation: " << std::endl<< T_result << std::endl;
  std::cout << "inverse: "<< std::endl << T_inverse << std::endl;
  //std::cout << "ICP Algorithm Information: " << std::endl;
  //std::cout <<  icp.getSearchMethodTarget() << std::endl;
  
  // This part below seems very over bloated !!! 
  // I feel like this is done in a method somewhere - manually converting from TF to EIGEN
  // the benefit is that the transformation matrix or quaternion is available as TF
  
  tf::Quaternion q_result;
  tf2::Quaternion *q_result_tf2 (new tf2::Quaternion);
  
  tf::Quaternion q_inverse;
  tf2::Quaternion *q_inverse_tf2 (new tf2::Quaternion);
  // instantiate a 3x3 rotation matrix from the transformation matrix 
  // I feel like this is done in a method somewhere
  
  tf::Matrix3x3 R_result( T_result(0,0),T_result(0,1),T_result(0,2),
                          T_result(1,0),T_result(1,1),T_result(1,2),
                          T_result(2,0),T_result(2,1),T_result(2,2));
  tf2::Matrix3x3 R_result_tf2(T_result(0,0),T_result(0,1),T_result(0,2),
                              T_result(1,0),T_result(1,1),T_result(1,2),
                              T_result(2,0),T_result(2,1),T_result(2,2));
  
  tf::Matrix3x3 R_inverse(T_inverse(0,0),T_inverse(0,1),T_inverse(0,2),
                          T_inverse(1,0),T_inverse(1,1),T_inverse(1,2),
                          T_inverse(2,0),T_inverse(2,1),T_inverse(2,2));
  tf2::Matrix3x3 R_inverse_tf2( T_inverse(0,0),T_inverse(0,1),T_inverse(0,2),
                                T_inverse(1,0),T_inverse(1,1),T_inverse(1,2),
                                T_inverse(2,0),T_inverse(2,1),T_inverse(2,2));
  
  // copy tf::quaternion from R_result to q_result
  R_result.getRotation(q_result);
  R_result_tf2.getRotation(*q_result_tf2);
  
  q_result_tf2->normalize(); // normalize the Quaternion 
  
  q_result_tf2->normalize(); // normalize the Quaternion (this is done twice for no reason)
  
  // copy tf::quaternion from R_result to q_result
  R_inverse.getRotation(q_inverse);
  R_inverse_tf2.getRotation(*q_inverse_tf2);
  q_inverse_tf2->normalize(); // normalize the Quaternion
  
  // set set rotation and origin of a quaternion for the tf transform object
  T_AB.setRotation(q_result);
  T_AB.setOrigin(tf::Vector3(T_result(0,3),T_result(1,3),T_result(2,3)));

  // set set rotation and origin of a quaternion for the tf transform object
  T_BA.setRotation(q_inverse);
  T_BA.setOrigin(tf::Vector3(T_inverse(0,3),T_inverse(1,3),T_inverse(2,3)));

  tf::transformStampedTFToMsg(T_AB,msg_AB);
  tf::transformStampedTFToMsg(T_BA,msg_BA);

  std::cout << "END OF REGISTER_CLOUD_ICP FUNCTION" << std::endl;
  return fit_score;
}

// define all template types that may be used with the function
template double CloudRegistration::registerCloudICP <pcl::PointXYZ>
                (pcl::PointCloud<pcl::PointXYZ> &source, pcl::PointCloud<pcl::PointXYZ> &target, 
                 tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
                 geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA);

template double CloudRegistration::registerCloudICP <pcl::PointXYZRGB>
                (pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, 
                 tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
                 geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA);

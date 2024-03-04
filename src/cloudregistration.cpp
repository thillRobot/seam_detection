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
//#include <tf/LinearMath/Matrix3x3.h>
//#include <tf_conversions/tf_eigen.h>
//#include <geometry_msgs/TransformStamped.h>
#include <pcl/registration/icp.h>



// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
// PCL PointCloud with XYZ RGB Normal Points
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

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
  
  node.getParam("input_file", input_file);    
  node.getParam("output_file", output_file);
 
} 

std::string CloudRegistration::getConfig(void){

  return config;

}


// function REGISTER_CLOUD_ICP finds the transform between two pointclouds using PCL::IterativeClosestPoint
double register_cloud_icp(PointCloud &source, PointCloud &target, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
                          geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, 
                          double max_corr_dist, double max_iter, double trns_epsl, double ecld_fitn_epsl, 
                          double ran_rej_thrsh, double e_results[],double c_offset[]){
  
  // get size of inputs clouds
  int Ns = source.size();
  int Nt = target.size();
  std::cout<<"BEGINNING ICP REGISTRATION" << std::endl;
  std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;
  
  std::cout<<"Using Search Parameters:"<< std::endl;
  std::cout<<"Max Correspondence Distance = "<< max_corr_dist <<std::endl;
  std::cout<<"Maximum Number of Iterations = "<< max_iter <<std::endl;
  std::cout<<"Transformation Epsilon = "<< trns_epsl <<std::endl;
  std::cout<<"Euclidean Distance Difference Epsilon = "<< ecld_fitn_epsl <<std::endl;
  
  // perform ICP on the lidar and cad clouds
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::PointCloud<pcl::PointXYZ> Final;
  
  Eigen::MatrixXf T_result, T_inverse;
  
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (max_corr_dist);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (max_iter);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (trns_epsl);
  // Set the euclidean distance difference epsilon (criterion 3)
  
  icp.setEuclideanFitnessEpsilon (ecld_fitn_epsl);
  // Set the RANSAC Outlier Rejection Threshold
  icp.setRANSACOutlierRejectionThreshold (ran_rej_thrsh);
  
  // these copies seem like a waste to me, figure out how to cut these out
  // make a copy of the LiDAR(source) cloud 
  PointCloud::Ptr src (new PointCloud);       //use this as the working copy of the source cloud
  pcl::copyPointCloud(source,*src);
  // make a copy of the CAD(target) cloud 
  PointCloud::Ptr tgt (new PointCloud);       //use this as the working copy of the target cloud
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
  // instantiate a 3x3 rotation matrix from the transformation matrix // I feel like this is done in a method somewhere
  
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

  return fit_score;
  std::cout << "END OF REGISTER_CLOUD_ICP FUNCTION" << std::endl;
}


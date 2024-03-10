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

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
//#include <teaser/point_cloud.h>
//#include <teaser/features.h>
 

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

template double CloudRegistration::registerCloudICP <pcl::PointXYZRGBNormal>
                (pcl::PointCloud<pcl::PointXYZRGBNormal> &source, pcl::PointCloud<pcl::PointXYZRGBNormal> &target, 
                 tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
                 geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA);


// templated function REGISTER_CLOUD_TEASER finds the transform between two pointclouds
// based on examples/teaser_cpp_ply.cc
template<typename point_t>
void CloudRegistration::registerCloudTeaser(pcl::PointCloud<point_t> &source, 
                                            pcl::PointCloud<point_t> &target, 
                                            tf::StampedTransform &T_AB, 
                                            tf::StampedTransform &T_BA, 
                                            geometry_msgs::TransformStamped &msg_AB, 
                                            geometry_msgs::TransformStamped &msg_BA, 
                                            double tparams[]){
  
   // get size of inputs clouds
   int Ns = source.size();
   int Nt = target.size();
   int P = 50; //number to print
   int M = -1; //number of matches
  
   std::cout <<"BEGINNING REGISTER_CLOUD_TEASER"<< std::endl;
   std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;
   if(Nt>Ns){ // if the target is larger, only use as many target points as are in the source
     Nt=Ns;
   } 
   
  // Convert the input point clouds to Eigen
   Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, Ns);
   Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, Nt);
    
   for (size_t i = 0; i < Ns; ++i) {
     src.col(i) << source[i].x, source[i].y, source[i].z;
   }
   for (size_t i = 0; i < Nt; ++i) {
     tgt.col(i) << target[i].x, target[i].y, target[i].z;
   }
    
   std::cout <<"Processing "<< src.size() << " source points and " <<tgt.size()<<" target points" << std::endl ;
   
// Run TEASER++ registration
   // Prepare solver parameters
   teaser::RobustRegistrationSolver::Params params;
   params.noise_bound = 0.05;
   params.cbar2 = 1;
   params.estimate_scaling = false;
   params.rotation_max_iterations = 100;
   params.rotation_gnc_factor = 1.4;
   params.rotation_estimation_algorithm =
       teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
   params.rotation_cost_threshold = 0.005;
 
   // Solve with TEASER++
   teaser::RobustRegistrationSolver solver(params);
   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

   solver.solve(src, tgt); 

   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
 
   auto soln = solver.getSolution();
 
   // Compare results
   std::cout << "TEASER++ Completed" << std::endl;
 
   //std::cout << "Expected rotation: " << std::endl;
   //std::cout << T.topLeftCorner(3, 3) << std::endl;
   std::cout << "Estimated rotation: " << std::endl;
   std::cout << soln.rotation << std::endl;
   //std::cout << "Error (deg): " << getAngularError(T.topLeftCorner(3, 3), solution.rotation)
   //         << std::endl;
   //std::cout << std::endl;
   //std::cout << "Expected translation: " << std::endl;
   //std::cout << T.topRightCorner(3, 1) << std::endl;
   std::cout << "Estimated translation: " << std::endl;
   std::cout << soln.translation << std::endl;
   //std::cout << "Error (m): " << (T.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
   //std::cout << std::endl;
   //std::cout << "Number of correspondences: " << N << std::endl;
   //std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
   std::cout << "Time taken (s): "
             << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                    1000000.0
             << std::endl;
 
   Eigen::MatrixXd soln_T(4,4); // a Transformation matrix for the teaser solution 
   soln_T<<soln.rotation(0,0),soln.rotation(0,1),soln.rotation(0,2),soln.translation(0),
           soln.rotation(1,0),soln.rotation(1,1),soln.rotation(1,2),soln.translation(1),
           soln.rotation(2,0),soln.rotation(2,1),soln.rotation(2,2),soln.translation(2),
           0                 ,0                 ,0                 ,1                  ;
 
   Eigen::MatrixXd soln_T_inv(4,4);
   soln_T_inv=soln_T.inverse(); // take the inverse of the transformation returned by Teaser
 
   // This part seems very over bloated !!! 
   // I feel like this is done in a method somewhere - manually converting from TF to EIGEN
 
   tf::Quaternion q_result;
   tf2::Quaternion *q_result_tf2 (new tf2::Quaternion);
 
   tf::Quaternion q_inverse;
   tf2::Quaternion *q_inverse_tf2 (new tf2::Quaternion);
   // instantiate a 3x3 rotation matrix from the transformation matrix // 
 
 
   tf::Matrix3x3 R_result(soln.rotation(0,0),soln.rotation(0,1),soln.rotation(0,2),
                          soln.rotation(1,0),soln.rotation(1,1),soln.rotation(1,2),
                          soln.rotation(2,0),soln.rotation(2,1),soln.rotation(2,2));
   tf2::Matrix3x3 R_result_tf2(soln.rotation(0,0),soln.rotation(0,1),soln.rotation(0,2),
                               soln.rotation(1,0),soln.rotation(1,1),soln.rotation(1,2),
                               soln.rotation(2,0),soln.rotation(2,1),soln.rotation(2,2));
 
   tf::Matrix3x3 R_inverse(soln_T_inv(0,0),soln_T_inv(0,1),soln_T_inv(0,2),
                           soln_T_inv(1,0),soln_T_inv(1,1),soln_T_inv(1,2),
                           soln_T_inv(2,0),soln_T_inv(2,1),soln_T_inv(2,2));
   tf2::Matrix3x3 R_inverse_tf2( soln_T_inv(0,0),soln_T_inv(0,1),soln_T_inv(0,2),
                                 soln_T_inv(1,0),soln_T_inv(1,1),soln_T_inv(1,2),
                                 soln_T_inv(2,0),soln_T_inv(2,1),soln_T_inv(2,2));
 
   // copy tf::quaternion from R_result to q_result
   R_result.getRotation(q_result);
   R_result_tf2.getRotation(*q_result_tf2);
   q_result_tf2->normalize(); // normalize the Quaternion
 
   // copy tf::quaternion from R_inverse to q_inverse
   R_inverse.getRotation(q_inverse);
   R_inverse_tf2.getRotation(*q_inverse_tf2);
   q_inverse_tf2->normalize(); // normalize the Quaternion
 
   // set rotation and origin of a quaternion for the tf transform object
   T_AB.setRotation(q_result);
   T_AB.setOrigin(tf::Vector3(soln.translation[0],soln.translation[1],soln.translation[2]));
 
   // set rotation and origin of a quaternion for the tf transform object
   T_BA.setRotation(q_inverse);
   T_BA.setOrigin(tf::Vector3(soln_T_inv(0,3),soln_T_inv(1,3),soln_T_inv(2,3)));
 
   tf::transformStampedTFToMsg(T_AB,msg_AB);
   tf::transformStampedTFToMsg(T_BA,msg_BA);
 
   std::cout << "END OF REGISTER_CLOUD_TEASER FUNCTION" << std::endl;
 
 }
 
template void CloudRegistration::registerCloudTeaser<pcl::PointXYZ>
              (pcl::PointCloud<pcl::PointXYZ> &source, pcl::PointCloud<pcl::PointXYZ> &target, 
               tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
               geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, 
               double tparams[]);

template void CloudRegistration::registerCloudTeaser<pcl::PointXYZRGB>
              (pcl::PointCloud<pcl::PointXYZRGB> &source, pcl::PointCloud<pcl::PointXYZRGB> &target, 
               tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
               geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, 
               double tparams[]);

template void CloudRegistration::registerCloudTeaser<pcl::PointXYZRGBNormal>
              (pcl::PointCloud<pcl::PointXYZRGBNormal> &source, pcl::PointCloud<pcl::PointXYZRGBNormal> &target, 
               tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, 
               geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, 
               double tparams[]);


// function REGISTER_CLOUD_TEASER finds the transform between two pointclouds, based on examples/teaser_cpp_ply.cc
template <typename point_t>
Eigen::Matrix<double, 6, Eigen::Dynamic> CloudRegistration::registerCloudTeaserFPFH(pcl::PointCloud<point_t> &source, 
                                                                                    pcl::PointCloud<point_t> &target, 
                                                                                    pcl::PointCloud<point_t> &corrs,  
                                                                                    tf::StampedTransform &T_AB, 
                                                                                    tf::StampedTransform &T_BA, 
                                                                                    geometry_msgs::TransformStamped &msg_AB, 
                                                                                    geometry_msgs::TransformStamped &msg_BA, 
                                                                                    double tparams[], 
                                                                                    teaser::FPFHEstimation features ){


  // get size of inputs clouds
  int Ns = source.size();
  int Nt = target.size();
  //int P = 50; //number to print
  //int M = -1; //number of matches
  std::cout <<"BEGINNING REGISTER_CLOUD_TEASER_FPFH"<< std::endl;
  std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;

  // instantiate teaser pointclouds
  teaser::PointCloud src;
  teaser::PointCloud tgt;

  for (size_t i = 0; i < Nt; ++i) {
    tgt.push_back({static_cast<float>(target[i].x), static_cast<float>(target[i].y), static_cast<float>(target[i].z)});
  }
  for (size_t i = 0; i < Ns; ++i) {
    src.push_back({static_cast<float>(source[i].x), static_cast<float>(source[i].y), static_cast<float>(source[i].z)});
  }

  // Compute FPFH (features)
  teaser::FPFHEstimation fpfh;
  auto obj_descriptors = fpfh.computeFPFHFeatures(src, 0.02, 0.04);
  auto scene_descriptors = fpfh.computeFPFHFeatures(tgt, 0.02, 0.04);

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
      src, tgt, *obj_descriptors, *scene_descriptors, false, true, false, 0.95);

  //std::vector<std::pair<int, int>> correspondences = matcher.calculateCorrespondences(
  //src, tgt, *obj_descriptors, *scene_descriptors, false, true, false, 0.95);
  //std::vector<std::pair<float, float>> corrs_points_pairs;

  int Nc=correspondences.size();
  Eigen::Matrix<double, 6, Eigen::Dynamic> corrs_points(6, Nc);
  //Eigen::Matrix<double, 3, Eigen::Dynamic> source_corrs_points(3, Nc);

  for(size_t i = 0; i < Nc; i++)
  {

    corrs_points.col(i) << source[correspondences[i].first].x, source[correspondences[i].first].y, source[correspondences[i].first].z,
                           target[correspondences[i].first].x, target[correspondences[i].first].y, target[correspondences[i].first].z;

  }

  //auto cloud_features = teaser::features::extract_fpfh(source);

  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.001;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 1000;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  solver.solve(src, tgt, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  auto solution = solver.getSolution();

  // results
  std::cout << "TEASER++ FPFH Completed" << std::endl;
  std::cout << "Estimated rotation: " << std::endl;
  std::cout << solution.rotation << std::endl;
  std::cout << std::endl;
  std::cout << "Estimated translation: " << std::endl;
  std::cout << solution.translation << std::endl;
  std::cout << "Number of correspondences: " << Nc << std::endl;
  //std::cout << "correspondences:" <<correspondences << std::endl;
  //std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                   1000000.0
            << std::endl;

  Eigen::MatrixXd solution_T(4,4); // a Transformation matrix for the teaser solution 
  solution_T<<solution.rotation(0,0),solution.rotation(0,1),solution.rotation(0,2),solution.translation(0),
              solution.rotation(1,0),solution.rotation(1,1),solution.rotation(1,2),solution.translation(1),
              solution.rotation(2,0),solution.rotation(2,1),solution.rotation(2,2),solution.translation(2),
              0                 ,0                 ,0                 ,1                  ;

  Eigen::MatrixXd solution_T_inv(4,4);
  solution_T_inv=solution_T.inverse(); // take the inverse of the transformation returned by Teaser

  // This part seems very over bloated !!! 
  // I feel like this is done in a method somewhere - manually converting from TF to EIGEN

  tf::Quaternion q_result;
  tf2::Quaternion *q_result_tf2 (new tf2::Quaternion);

  tf::Quaternion q_inverse;
  tf2::Quaternion *q_inverse_tf2 (new tf2::Quaternion);
  // instantiate a 3x3 rotation matrix from components of the transformation matrix // 
  tf::Matrix3x3 R_result(solution.rotation(0,0),solution.rotation(0,1),solution.rotation(0,2),
                         solution.rotation(1,0),solution.rotation(1,1),solution.rotation(1,2),
                         solution.rotation(2,0),solution.rotation(2,1),solution.rotation(2,2));
  tf2::Matrix3x3 R_result_tf2(solution.rotation(0,0),solution.rotation(0,1),solution.rotation(0,2),
                              solution.rotation(1,0),solution.rotation(1,1),solution.rotation(1,2),
                              solution.rotation(2,0),solution.rotation(2,1),solution.rotation(2,2));

  tf::Matrix3x3 R_inverse(solution_T_inv(0,0),solution_T_inv(0,1),solution_T_inv(0,2),
                          solution_T_inv(1,0),solution_T_inv(1,1),solution_T_inv(1,2),
                          solution_T_inv(2,0),solution_T_inv(2,1),solution_T_inv(2,2));
  tf2::Matrix3x3 R_inverse_tf2( solution_T_inv(0,0),solution_T_inv(0,1),solution_T_inv(0,2),
                                solution_T_inv(1,0),solution_T_inv(1,1),solution_T_inv(1,2),
                                solution_T_inv(2,0),solution_T_inv(2,1),solution_T_inv(2,2));

  // copy tf::quaternion from R_result to q_result
  R_result.getRotation(q_result);
  R_result_tf2.getRotation(*q_result_tf2);
  q_result_tf2->normalize(); // normalize the Quaternion

  // copy tf::quaternion from R_inverse to q_inverse
  R_inverse.getRotation(q_inverse);
  R_inverse_tf2.getRotation(*q_inverse_tf2);
  q_inverse_tf2->normalize(); // normalize the Quaternion

  // set rotation and origin of a quaternion for the tf transform object
  T_AB.setRotation(q_result);
  T_AB.setOrigin(tf::Vector3(solution.translation[0],solution.translation[1],solution.translation[2]));

  // set rotation and origin of a quaternion for the tf transform object
  T_BA.setRotation(q_inverse);
  T_BA.setOrigin(tf::Vector3(solution_T_inv(0,3),solution_T_inv(1,3),solution_T_inv(2,3)));

  tf::transformStampedTFToMsg(T_AB,msg_AB);
  tf::transformStampedTFToMsg(T_BA,msg_BA);

  std::cout << "REGISTER_CLOUD_TEASER_FPFH Complete" << std::endl;

  return corrs_points;

}

template Eigen::Matrix<double, 6, Eigen::Dynamic> CloudRegistration::registerCloudTeaserFPFH<pcl::PointXYZ>
                                                                         (pcl::PointCloud<pcl::PointXYZ> &source, 
                                                                          pcl::PointCloud<pcl::PointXYZ> &target,
                                                                          pcl::PointCloud<pcl::PointXYZ> &corrs,
                                                                          tf::StampedTransform &T_AB, 
                                                                          tf::StampedTransform &T_BA,
                                                                          geometry_msgs::TransformStamped &msg_AB, 
                                                                          geometry_msgs::TransformStamped &msg_BA,
                                                                          double tparams[], teaser::FPFHEstimation features);

template Eigen::Matrix<double, 6, Eigen::Dynamic> CloudRegistration::registerCloudTeaserFPFH<pcl::PointXYZRGB>
                                                                         (pcl::PointCloud<pcl::PointXYZRGB> &source, 
                                                                          pcl::PointCloud<pcl::PointXYZRGB> &target,
                                                                          pcl::PointCloud<pcl::PointXYZRGB> &corrs,
                                                                          tf::StampedTransform &T_AB, 
                                                                          tf::StampedTransform &T_BA,
                                                                          geometry_msgs::TransformStamped &msg_AB, 
                                                                          geometry_msgs::TransformStamped &msg_BA,
                                                                          double tparams[], teaser::FPFHEstimation features);

template Eigen::Matrix<double, 6, Eigen::Dynamic> CloudRegistration::registerCloudTeaserFPFH<pcl::PointXYZRGBNormal>
                                                                         (pcl::PointCloud<pcl::PointXYZRGBNormal> &source, 
                                                                          pcl::PointCloud<pcl::PointXYZRGBNormal> &target,
                                                                          pcl::PointCloud<pcl::PointXYZRGBNormal> &corrs,
                                                                          tf::StampedTransform &T_AB, 
                                                                          tf::StampedTransform &T_BA,
                                                                          geometry_msgs::TransformStamped &msg_AB, 
                                                                          geometry_msgs::TransformStamped &msg_BA,
                                                                          double tparams[], teaser::FPFHEstimation features);



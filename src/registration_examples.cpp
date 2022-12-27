/*
This source contains the several examples of point cloud registration
for testing and demonstration purposes

Tristan Hill - Weld Seam Detection - Tennessee Technological University
Robotics Research Group - Mechanical Engineering

v1.0 - 2022-12-20 

see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/pcl_config.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "ros/package.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
//#include <tf/TransformStamped.h>

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
//#include <teaser/point_cloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;


// This function REGISTER_CLOUD_ICP finds the transform between two pointclouds using PCL::IterativeClosestPoint
void register_cloud_icp(PointCloud &target, PointCloud &source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double max_corr_dist, double max_iter, double trns_epsl, double ecld_fitn_epsl, double e_results[],double c_offset[])
{
 
  std::cout<<"BEGINNING ICP REGISTRATION" << std::endl;
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

  // these copies seem like a waste to me, figure out how to cut these out
  // make a copy of the CAD(target) cloud called 'cloud_A' 
  PointCloud::Ptr tgt (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(target,*tgt);
  // make a copy of the LiDAR(source) cloud called 'cloud_B'
  PointCloud::Ptr src (new PointCloud);       //use this as the working copy of the source cloud
  pcl::copyPointCloud(source,*src);

  icp.setInputTarget(tgt); // target (fixed) cloud
  icp.setInputSource(src); // source (moved during ICP) cloud
  icp.align(Final); // perform ICP registration

  T_result=icp.getFinalTransformation(); // get the resutls of ICP
  T_inverse=T_result.inverse();          // also store the inverse transformation

  std::cout << "ICP COMPLETED" << std::endl;
  std::cout << "max iterations:" << icp.getMaximumIterations() << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
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

  q_result_tf2->normalize(); // normalize the Quaternion

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
}


// This function REGISTER_CLOUD_TEASER finds the transform between two pointclouds, based on examples/teaser_cpp_ply.cc
void register_cloud_teaser(PointCloud &target, PointCloud &source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double tparams[])
{
 
  // get size of inputs clouds
  int Ns = source.size();
  int Nt = target.size();
  int P = 50; //number to print
  int M = -1; //number of matches
  std::cout <<"BEGINNING REGISTER_CLOUD_TEASER"<< std::endl;
  std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;

  // instantiate teaser pointclouds
  //teaser::PointCloud src_cloud;
  //teaser::PointCloud tgt_cloud;
  
  // Convert the input point clouds to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, Ns);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, Nt);
  
  for (size_t i = 0; i < Ns; ++i) {
    src.col(i) << source[i].x, source[i].y, source[i].z;
  }  
  for (size_t i = 0; i < Nt; ++i) {
    tgt.col(i) << target[i].x, target[i].y, target[i].z;
  }
  
  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.05;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 10000;
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


// This function REGISTER_CLOUD_TEASER finds the transform between two pointclouds, based on examples/teaser_cpp_ply.cc
void register_cloud_teaser_fpfh(PointCloud &target, PointCloud &source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double tparams[])
{
 
  // get size of inputs clouds
  int Nt = target.size();
  int Ns = source.size();
  int P = 50; //number to print
  int M = -1; //number of matches
  std::cout <<"BEGINNING REGISTER_CLOUD_TEASER_FPFH"<< std::endl;
  std::cout <<"Processing "<< Nt << " target points and " <<Ns<<" source points" << std::endl ;

  // instantiate teaser pointclouds
  teaser::PointCloud tgt;
  teaser::PointCloud src;
  
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
  std::cout << "Number of correspondences: " << Ns << std::endl;
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

}


// this function calculates a difference detween the measured and expected transformation and prints the info to the console
void analyze_results(tf::Transform &tf_in,double e_results[])
{
  
  std::cout<<"Measured Rotation Matrix:"<<std::endl;  
  std::cout<<"["<<tf_in.getBasis()[0][0]<<","<<tf_in.getBasis()[0][1]<<","<<tf_in.getBasis()[0][2]<<","<<std::endl;
  std::cout     <<tf_in.getBasis()[1][0]<<","<<tf_in.getBasis()[1][1]<<","<<tf_in.getBasis()[1][2]<<","<<std::endl;
  std::cout     <<tf_in.getBasis()[2][0]<<","<<tf_in.getBasis()[2][1]<<","<<tf_in.getBasis()[2][2]<<"]"<<std::endl;

  std::cout<<"Expected,Translation: ["<<e_results[0]<<","
                                      <<e_results[1]<<","
                                      <<e_results[2]<<"]"<<std::endl;
  std::cout<<"Measured Translation: ["<<tf_in.getOrigin().getX()<<","
                                      <<tf_in.getOrigin().getY()<<","
                                      <<tf_in.getOrigin().getZ()<<"]"<<std::endl;
  std::cout<<"Difference Translation: ["<<e_results[0]-tf_in.getOrigin().getX()<<","
                                      <<e_results[1]-tf_in.getOrigin().getY()<<","
                                      <<e_results[2]-tf_in.getOrigin().getZ()<<"]"<<std::endl;

  std::cout<<"Expected Rotation: [" <<e_results[3]<<","
                                    <<e_results[4]<<","
                                    <<e_results[5]<<"]"<<std::endl;
  std::cout<<"Measured Rotation: [" <<tf_in.getRotation().getAxis().getX()
                                    <<","<<tf_in.getRotation().getAxis().getY()
                                    <<","<<tf_in.getRotation().getAxis().getZ()<<"]"<<std::endl; 
  std::cout<<"Difference Rotation: [" <<e_results[3]-tf_in.getRotation().getAxis().getX()
                                    <<","<<e_results[4]-tf_in.getRotation().getAxis().getY()
                                    <<","<<e_results[5]-tf_in.getRotation().getAxis().getZ()<<"]"<<std::endl; 

  //std::cout<<"W:"<<tf_in.getRotation().getW()<<std::endl;

}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"registration_examples");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Registration Examples v1.0                      "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Loading Configuration File                      "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // there is only one cmd line arg and it is the name of the config file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // parameters that contain strings  
  std::string source_cloud_path, target_cloud_path, source_cloud_file, target_cloud_file;

  node.getParam("lidar_src_file", source_cloud_file);
  source_cloud_path=packagepath+'/'+source_cloud_file;

  node.getParam("cad_ref_file", target_cloud_file);
  target_cloud_path=packagepath+'/'+target_cloud_file;


  // parameters that contain doubles
  double voxel_leaf_size, ransac_norm_dist_wt, ransac_max_iter, ransac_dist_thrsh, ransac_k_srch,
         icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl;

  // parameters that contain vectors of doubles
  std::vector<double> xs, ys, zs, filter_box_vec, ransac_init_norm_vec, expected_results_vec, calibration_offset_vec, seam1_points_x_vec, seam1_points_y_vec, seam1_points_z_vec;
  double filter_box[6],ransac_init_norm[3],icp_params[4],expected_results[6],calibration_offset[6],seam1_points_x[4],seam1_points_y[4],seam1_points_z[4];
  
  node.getParam("filter_box",  filter_box_vec);
  for(unsigned i=0; i < filter_box_vec.size(); i++)
    filter_box[i]=filter_box_vec[i]; // copy from vector to array 
  node.getParam("voxel_leaf_size", voxel_leaf_size);

  node.getParam("ransac_norm_dist_wt",ransac_norm_dist_wt);  
  node.getParam("ransac_max_iter",ransac_max_iter);  
  node.getParam("ransac_dist_thrsh",ransac_dist_thrsh);  
  node.getParam("ransac_k_srch",ransac_k_srch);  
  node.getParam("ransac_init_norm",  ransac_init_norm_vec);
  for(unsigned i=0; i < ransac_init_norm_vec.size(); i++)
    ransac_init_norm[i]=ransac_init_norm_vec[i]; // copy from vector to array
  
  node.getParam("icp_max_corr_dist",icp_max_corr_dist);  // these four ICP parameters define the search
  node.getParam("icp_max_iter",icp_max_iter);
  node.getParam("icp_trns_epsl",icp_trns_epsl);
  node.getParam("icp_ecld_fitn_epsl",icp_ecld_fitn_epsl);

  node.getParam("expected_results",expected_results_vec);  // these four ICP parameters define the search
  node.getParam("calibration_offset",calibration_offset_vec);  // these four ICP parameters define the search
  for(unsigned i=0; i < expected_results_vec.size(); i++){
    expected_results[i]=expected_results_vec[i]; // copy into an array 
    calibration_offset[i]=calibration_offset_vec[i]; // copy into an array 
  }
  
  node.getParam("seam1_points_x",seam1_points_x_vec);
  node.getParam("seam1_points_y",seam1_points_y_vec);
  node.getParam("seam1_points_z",seam1_points_z_vec);
  for(unsigned i=0; i < seam1_points_x_vec.size(); i++){
    seam1_points_x[i]=seam1_points_x_vec[i]; // copy into arrays
    seam1_points_y[i]=seam1_points_y_vec[i]; 
    seam1_points_z[i]=seam1_points_z_vec[i];
  }

  // setup a tf for a 'searchbox' marker so we we can see it in RVIZ - maybe someday...
  // static tf::TransformBroadcaster br_searchbox;
  // tf::Transform tf_searchbox;

  std::cout<<"Debug0"<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Preparing Pointcloud Data                       "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // instantiate cloud objects
  PointCloud::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud
  PointCloud::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);  // target cloud  
  PointCloud::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZ>);  // alinged source cloud (using registration results)

  // load the cloud data from PCD files , files generated with src/cad_cloud.cpp
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (source_cloud_path, *source_cloud) == -1)
  {
    std::cout<<"Couldn't read image file:"<<source_cloud_path;
    return (-1);
  }else{
    std::cout << "Loaded "<<target_cloud->size()<< " data points from "<< target_cloud_file <<std::endl;
  }
  // load the cloud from CAD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_cloud_path, *target_cloud) == -1)
  {
    std::cout<<"Couldn't read image file:"<<target_cloud_path;
    return (-1);
  }else{
    std::cout << "Loaded "<<source_cloud->size()<< " data points from "<< source_cloud_file <<std::endl;
  }

  // for now each tf has three objects associated with it
  // 1) '<name>' (tf::transform)      // needed for transforms with pcl_ros
  // 2) '<name>_tf2' (tf2::transform) // not used
  // 3) '<name>_msg' (geometry_msgs)  // needed for bradcasting frames

  tf::StampedTransform *T_01 (new tf::StampedTransform);    // these are from the old 'TF'
  tf::StampedTransform *T_10 (new tf::StampedTransform);    // they are stil used for pcl_ros::transformPointCloud

  static tf2_ros::StaticTransformBroadcaster static_broadcaster; // this is the new 'TF2' way to broadcast tfs
  geometry_msgs::TransformStamped *T_01_msg (new geometry_msgs::TransformStamped);
  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";

  geometry_msgs::TransformStamped *T_10_msg (new geometry_msgs::TransformStamped);
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";


  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Processing Pointcloud Data                      "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // Perform ICP Cloud Registration to find location and orientation of part of interest
  register_cloud_icp(*source_cloud,*target_cloud,*T_10, *T_01, *T_10_msg, *T_01_msg, icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl,expected_results,calibration_offset);

  int N_cor=100;
  EigenCor cor_src_pts, cor_tgt_pts;

  // Perform TEASER++ cloud registration
  double teaser_params[3]={1,2,3}; // temporary place holder 
  register_cloud_teaser(*target_cloud, *source_cloud, *T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);

    // Perform TEASER++ cloud registration with Fast Point Feature Histograms (FPFH) descriptors  
  //double teaser_params[3]={1,2,3}; // temporary place holder 
  register_cloud_teaser_fpfh(*target_cloud, *source_cloud, *T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);

  // now align the CAD part to using the resulting transformation
  pcl_ros::transformPointCloud(*source_cloud, *aligned_cloud, *T_10); // this works with 'pcl::PointCloud<pcl::PointXYZ>' and 'tf::Transform'
  std::cout << "Cloud aligned using resulting transformation." << std::endl;
 
  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Analzing Results                                "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  analyze_results(*T_10, expected_results);
  analyze_results(*T_01, expected_results);

  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";

  
  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Preparing Visualization                         "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  ros::Publisher source_pub = node.advertise<PointCloud> ("/source_cloud", 1);
  ros::Publisher target_pub = node.advertise<PointCloud> ("/target_cloud", 1);
  ros::Publisher aligned_pub = node.advertise<PointCloud> ("/aligned_cloud", 1);

  source_cloud->header.frame_id = "base_link";
  target_cloud->header.frame_id = "base_link";
  aligned_cloud->header.frame_id = "base_link";

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    SEAM_DETECTION Complete                         "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  //publish forever
  while(ros::ok())
  {

      // this is the new 'TF2' way of broadcasting tfs
      T_01_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_msg);
      T_10_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_msg);

      source_pub.publish(source_cloud);
      target_pub.publish(target_cloud);
      aligned_pub.publish(aligned_cloud);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}





/*
RANSAC/Segementation based multiple plane detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University
Robotics Research Group - Mechanical Engineering

began with PCL sample code - 02/14/2018 see https://pointclouds.org/ for offical documentation
Updated - 02/17/2018
Revisited 02/22/2020
v1.0 - 12/07/2020 this officially became seam_detection_v1.0
v1.1 - 12/26/2020
...
v1.4 - 02/08/2021

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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

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
#include "ros/package.h"

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <chrono>
#include <random>

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

void filter_cloud(PointCloud &cloud_input, PointCloud &cloud_output,double xmin, double xmax, double ymin,double ymax, double zmin, double zmax, double leaf_size)
{

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_input,*cloud);
  std::cout << "BEGINNING CLOUD FILTERING" << std::endl;
  std::cout<<"Before filtering there are "<<cloud->width * cloud->height << " data points in the lidar cloud. "<< std::endl;

  // XYZ Box Filter cloud before segementation
  pcl::PassThrough<pcl::PointXYZ> pass;cloud_input,
  pass.setInputCloud(cloud);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(xmin,xmax);
  //pass.setFilterLimits(0.0,0.5);
  pass.filter (*cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(ymin,ymax);
  //pass.setFilterLimits(0.0,0.5);
  pass.filter (*cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits(zmin,zmax);
  //pass.setFilterLimits(0.01,0.5);
  pass.filter (*cloud);

  std::cout<<"Box/XYZ Filter Limits: [" <<xmin<<","<<xmax<<","<<ymin<<","<<ymax<<","<<zmin<<","<<zmax<<"]"<< std::endl;
  std::cout<<"After box/XYZ filtering there are "<<cloud->width * cloud->height << " data points in the lidar cloud. "<< std::endl;


  // Apply Voxel Filter the Cloud
  // use "001f","001f","0001f" or "none" to set voxel leaf size


  if (leaf_size>0)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (leaf_size, leaf_size, leaf_size);
    vox.filter (*cloud);
    std::cout<<"After voxel filtering there are "<<cloud->width * cloud->height << " data points in the lidar cloud. "<< std::endl;
  }else
  {
    std::cout<<"No voxel filtering"<< std::endl;
  }

  pcl::copyPointCloud(*cloud,cloud_output);

}


void segment_cloud(PointCloud &cloud_input, PointCloud &cloud_output1, PointCloud &cloud_output2, PointCloud &cloud_output3, const std::string& part1_type, pcl::ModelCoefficients::Ptr C_plane, pcl::ModelCoefficients::Ptr C_cylinder)
{

  // instantiate all objects needed for segment_cloud function
  //pcl::PCDReader reader;
  //pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  //pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets - local to this function
  //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud_squaretube (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

  // Apply Workspace Filter using XYZ and Voxel filters before performing segmentation
  filter_cloud(cloud_input,*cloud_filtered, -0.2, 0.4, -0.2, 0.4, -0.30, 0.50, 0.0005);

  std::cout << "BEGINNING RANSAC SEGMENTATION" << std::endl;
  
  std::cout<<"Performing First Segmentaion"<<std::endl;
  std::cout<<"Searching for plate/table as: SACMODEL_NORMAL_PLANE"<< std::endl;; // as a single plane?

  // Estimate point normals before segmentation
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Perform RANSAC Segmentation to find plane first
  // Instantiate segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  *C_plane=*coefficients_plane;
  std::cout << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  extract.filter (*cloud_plane);

  std::cout << "The PointCloud representing the planar component contains: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2); // cloud filtered2 are the outliers of the plane segmentation
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Copy the filtered cloud as third output
  pcl::copyPointCloud(*cloud_filtered,cloud_output3);

  // Copy the plane inliers as cloud for part 2 (plate or table)
  pcl::copyPointCloud(*cloud_plane,cloud_output2);

  // Apply Box and Voxel filters before performing second segmentation
  // zmin=~0.3 here should be automatically set by first segementation using the z value of the plane
  filter_cloud(*cloud_filtered2,*cloud_filtered3, -0.2, 0.4, -0.2, 0.4, 0.00, 0.5, -1);

  if (part1_type=="round_tube") //part two is a cylinder - this variable is set by command lines args
  {

    std::cout<<"Performing Second Segmentaion"<<std::endl;
    std::cout<<"Searching for round-tube as a cylinder as: SACMODEL_CYLINDER"<<std::endl;

    // Estimate point normals before segmentation
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered3);
    ne.setKSearch (50);
    ne.compute (*cloud_normals3);

    // Create segmentation object for cylinder segmentation and set parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered3);
    seg.setInputNormals (cloud_normals3);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    *C_cylinder=*coefficients_cylinder;
    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered3);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);

    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
      std::cout << "Cannot find a cylindrical component in cloud" << std::endl;
    else
    {
      std::cout << "The PointCloud representing the cylindrical component contains: " << cloud_cylinder->points.size () << " data points." << std::endl;
    }

    pcl::copyPointCloud(*cloud_cylinder,cloud_output1);    // use second segmentation

  }else if (part1_type=="square_tube")
  {

    std::cout<<"Performing Second Segmentaion"<<std::endl;
    std::cout<<"Searching for square-tube as: <INSERT SACMODEL>"<< std::endl;; // as a single plane?
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered3);
    ne.setKSearch (50);
    ne.compute (*cloud_normals3);
    // Create the segmentation object for ??? segmentation and set the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered3);
    seg.setInputNormals (cloud_normals3);

    // Obtain the square-tube inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    *C_cylinder=*coefficients_cylinder;
    std::cout << "Square Tube coefficients : " << *coefficients_cylinder << std::endl;

    // Write the square-tube inliers to disk
    extract.setInputCloud (cloud_filtered3);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);

    extract.filter (*cloud_squaretube);
    if (cloud_squaretube->points.empty ())
      std::cout << "Cannot find the square tube component of cloud." << std::endl;
    else
    {
      std::cout << "PointCloud representing the square tube component: " << cloud_squaretube->points.size () << " data points." << std::endl;
      //writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }

    pcl::copyPointCloud(*cloud_squaretube,cloud_output1);  // use second segmentation

  }else if (part1_type=="generic")
  {
    std::cout<<"Skipping Second Segmentation"<<std::endl;
    std::cout<<"Proceeding with filtered outliers of first segmentation"<<std::endl;
    pcl::copyPointCloud(*cloud_filtered3,cloud_output1); // ignore second segmentation  
  }

  std::cerr << "END OF SEGMENT_CLOUD FUNCTION" << std::endl;

}

// This function REGISTER_CLOUD finds the transform between two pointclouds
void register_cloud_icp(PointCloud &cloud_target, PointCloud &cloud_source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double params[])
{
 
  // make 2 copy of the lidar cloud called 'cloud_A' and 'cloud_B'
  PointCloud::Ptr cloud_A (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_target,*cloud_A);
  // make a copy of the lidar cloud called 'cloud'
  PointCloud::Ptr cloud_B (new PointCloud);       //use this as the working copy of the source cloud
  pcl::copyPointCloud(cloud_source,*cloud_B);

  // perform ICP on the lidar and cad clouds
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::PointCloud<pcl::PointXYZ> Final;

  Eigen::MatrixXf T_result;
  Eigen::MatrixXf T_inverse;

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (params[0]);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (params[1]);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (params[2]);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (params[3]);

  icp.setInputTarget(cloud_A); // target (fixed) cloud
  icp.setInputCloud(cloud_B);  // source (moved during ICP) cloud
  icp.align(Final);

  T_result=icp.getFinalTransformation(); // get the resutls of ICP
  T_inverse=T_result.inverse();

  std::cout << "ICP COMPLETED" << std::endl;
  std::cout << "max iterations:" << icp.getMaximumIterations() << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << T_result << std::endl;

  // This part seems very over bloated !!! 
  // I feel like this is done in a method somewhere - manually converting from TF to EIGEN

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

// This function REGISTER_CLOUD finds the transform between two pointclouds
void register_cloud_teaser(PointCloud &cloud_target, PointCloud &cloud_source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double params[])
{
 
  // make 2 copy of the lidar cloud called 'cloud_A' and 'cloud_B'
  //PointCloud::Ptr target (new PointCloud);       //use this as the working copy of the target cloud
  //pcl::copyPointCloud(cloud_target,*target);
  //// make a copy of the lidar cloud called 'cloud'
  //PointCloud::Ptr source (new PointCloud);       //use this as the working copy of the source cloud
  //pcl::copyPointCloud(cloud_source,*source);

  std::cout <<"Beginning Correspondence Estimation with PCL"<< std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> estimator;
  estimator.setInputCloud (source);
  estimator.setInputTarget (target);
  estimator.determineReciprocalCorrespondences (*correspondences);

 // check for correct order and number of matches

  //if (int (correspondences->size ()) == nr_original_correspondences)
  //{
  int imax;
  std::cout <<"Correspondence Estimation Complete"<< std::endl;
  std::cout <<"Index, Index Query, Index Match"<< std::endl;
  for (int i = 0; i < 40; ++i)
    std::cout <<i<<","<<(*correspondences)[i].index_query<<","<<(*correspondences)[i].index_match<< std::endl;

    // check for correct matches
    //for (int i = 0; i < nr_original_correspondences; ++i)
    //  std::cout <<((*correspondences)[i].index_match<<"," i << std::endl;

  //}

  //pcl::PointCloud<pcl::PointXYZ>::Ptr source, target;
  //pcl::copyPointCloud(cloud_target,*target);
  //pcl::copyPointCloud(cloud_source,*source);

  // ... read or fill in source and target
  //pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> estimator;
  //estimator.setInputSource (source);
  //est.setInputTarget (target);
 
  //pcl::Correspondences all_correspondences;
  // Determine all reciprocal correspondences
  //est.determineReciprocalCorrespondences (all_correspondences);


  pcl::registration::CorrespondenceRejectorSurfaceNormal rejector;
  //rejector.setInputTarget(target);
  //rejector.setInputSource(source);
  std::cout <<"CONVERTING CORRESPONDENCE POINTCLOUDS TO EIGEN" << std::endl;
  //int N = cloud_source.size();
  int N = 50;
  int S,T;
  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);
  for (size_t i = 0; i < N; ++i) {
    S=(*correspondences)[i].index_query;
    T=(*correspondences)[i].index_match;
    src.col(i) << cloud_source[S].x, cloud_source[S].y, cloud_source[S].z;
    tgt.col(i) << cloud_target[T].x, cloud_target[T].y, cloud_target[T].z;
  }

  /*
  int M = cloud_target.size();
  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, M);
  for (size_t i = 0; i < M; ++i) {
    tgt.col(i) << cloud_target[i].x, cloud_target[i].y, cloud_target[i].z;
  }
  */

  /*
  std::cout <<"CONVERTING POINTCLOUDS TO EIGEN" << std::endl;
  int N = cloud_source.size();
  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  for (size_t i = 0; i < N; ++i) {
    src.col(i) << cloud_source[i].x, cloud_source[i].y, cloud_source[i].z;
  }
  int M = cloud_target.size();
  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, M);
  for (size_t i = 0; i < M; ++i) {
    tgt.col(i) << cloud_target[i].x, cloud_target[i].y, cloud_target[i].z;
  }
  */

  //std::cout << "Source Cloud: "<<N<<" points"<< std::endl;
  //std::cout << "Target Cloud: "<<M<<" points"<< std::endl;


   // Run TEASER++ registration
  // Prepare solver parameters
  std::cout <<"Configuring TEASER++" << std::endl;
  teaser::RobustRegistrationSolver::Params tparams;
  tparams.noise_bound = NOISE_BOUND;
  tparams.cbar2 = 1;
  tparams.estimate_scaling = false;
  tparams.rotation_max_iterations = 100;
  tparams.rotation_gnc_factor = 1.4;
  tparams.rotation_estimation_algorithm =
  teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  tparams.rotation_cost_threshold = 0.005;

  // Solve with TEASER++
  std::cout <<"Beginning TEASER++" << std::endl;
  teaser::RobustRegistrationSolver solver(tparams);
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  solver.solve(src, tgt);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  auto soln = solver.getSolution();

  // Display results
  std::cout << "TEASER++ Completed" << std::endl;
  std::cout << "Expected rotation: " << std::endl;
  //std::cout << T.topLeftCorner(3, 3) << std::endl;
  std::cout << "Estimated rotation: " << std::endl;
  std::cout << soln.rotation << std::endl;
  std::cout << soln.rotation(0,0) << std::endl;
  //std::cout << "Error (deg): " << getAngularError(T.topLeftCorner(3, 3), solution.rotation)
  //          << std::endl;
  std::cout << std::endl;
  std::cout << "Expected translation: " << std::endl;
  //std::cout << T.topRightCorner(3, 1) << std::endl;
  std::cout << "Estimated translation: " << std::endl;
  std::cout << soln.translation << std::endl;
  //std::cout << "Error (m): " << (T.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
  std::cout << std::endl;
  std::cout << "Number of correspondences: " << N << std::endl;
  std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
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

void combine_transformation(tf::StampedTransform &T_AB, tf::StampedTransform &T_BC, tf::StampedTransform &T_AC, tf::StampedTransform &T_CA, geometry_msgs::TransformStamped &msg_AC,geometry_msgs::TransformStamped &msg_CA){

  tf::Transform T;
  tf::Transform T_inv;

  T=T_BC*T_AB;    // this operation (StampedTransform)*(StampedTransform) returns a (Transform) NOT a (StampedTransform)!! unstamped!
  T_inv=T.inverse();

  T_AC.setOrigin(T.getOrigin());
  T_AC.setRotation(T.getRotation());

  T_CA.setOrigin(T_inv.getOrigin());
  T_CA.setRotation(T_inv.getRotation());

  tf::transformStampedTFToMsg(T_AC,msg_AC);
  tf::transformStampedTFToMsg(T_CA,msg_CA);

}

// this function prints the info in a TF to the console
void print_tf(tf::Transform &tf_in)
{

  std::cout<<"Printing Transformation"<<std::endl;
  std::cout<<"Quaternion"<<std::endl;
  std::cout<<"Origin"<<std::endl;
  std::cout<<"X:"<<tf_in.getOrigin().getX()<<std::endl;
  std::cout<<"Y:"<<tf_in.getOrigin().getY()<<std::endl;
  std::cout<<"Z:"<<tf_in.getOrigin().getY()<<std::endl;
  std::cout<<"Axis"<<std::endl;
  std::cout<<"X:"<<tf_in.getRotation().getAxis().getX()<<std::endl;
  std::cout<<"Y:"<<tf_in.getRotation().getAxis().getY()<<std::endl;
  std::cout<<"Z:"<<tf_in.getRotation().getAxis().getZ()<<std::endl;
  std::cout<<"W:"<<tf_in.getRotation().getW()<<std::endl;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"******************** Seam Detection v1.4 ********************"<<endl;
  std::cout<<"*************************************************************"<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  // read the command line arguments to pick the data file and some other details
  
  // there is onlt one cmd line arg and it is the name of the config file
  
  // find the path to the seam_detection package (this package)

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"**************** Loading Configuration File ****************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;
  std::string packagepath = ros::package::getPath("seam_detection");

  std::string file_lidar; // source cloud
  std::string file_cad;// reference cloud
  std::string part1_type; // part1 is the part to be welded to plate

  std::string param1;
  node.getParam("scene_file", param1);
  file_lidar=packagepath+'/'+param1;

  std::string param2;
  node.getParam("part1_file", param2);
  file_cad=packagepath+'/'+param2;

  std::string param3;
  node.getParam("part1_type", param3);
  part1_type=param3;

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<double> icps;

  double icp_params[4];

  node.getParam("seam1_xs",xs); // these arrays define x,y,z, points in the model
  node.getParam("seam1_ys",ys);
  node.getParam("seam1_zs",zs);

  node.getParam("icp_params",icps);  // these four ICP parameters define the search

  // Define array for the four ICP search paremeters
  for(unsigned i=0; i < xs.size(); i++) {
    //std::cout<<xs[i]<<ys[i]<<zs[i]<<std::endl;
    icp_params[i]=icps[i]; // copy into an array to be used in register_cloud fn
  }


  //double sum = 0;
  //nh.getParam("my_double_list", my_double_list);

  // setup a tf for a 'searchbox' marker so we we can see it in RVIZ - maybe someday...
  // static tf::TransformBroadcaster br_searchbox;
  // tf::Transform tf_searchbox;

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"******************* Preparing Pointcloud Data ***************"<<endl;
  std::cout<<"*************************************************************"<<endl;

  // instantiate some clouds
  PointCloud::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZ>); // target cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); // target cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_cad1 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud
  PointCloud::Ptr cloud_cad2 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud intermediate
  PointCloud::Ptr cloud_cad3 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud final
  PointCloud::Ptr cloud_part1 (new pcl::PointCloud<pcl::PointXYZ>); // cylinder cloud // input to ICP
  PointCloud::Ptr cloud_part2 (new pcl::PointCloud<pcl::PointXYZ>); // plane cloud

  // load the clouds from file (.pcd)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_lidar, *cloud_lidar) == -1)
  {
      std::cout<<"Couldn't read image file:"<<file_lidar;
      return (-1);
  }
  std::cout << "Loaded image file: "<< file_lidar <<std::endl<<
      cloud_lidar->width * cloud_lidar->height << " Data points from "<< file_lidar << std::endl;

  // load the cloud from CAD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_cad, *cloud_cad1) == -1)
  {
      std::cout<<"Couldn't read image file:"<<file_cad;
      return (-1);
  }
  std::cout << "Loaded image file: "<< file_cad <<std::endl<<
      cloud_cad1->width * cloud_cad1->height << " Data points from "<< file_cad <<std::endl<<std::endl;

  // for now each tf has three objects associated with it
  // 1) '<name>' (tf::transform)      // needed for transforms with pcl_ros
  // 2) '<name>_tf2' (tf2::transform) // not used
  // 3) '<name>_msg' (geometry_msgs)  // needed for bradcasting frames

  tf::StampedTransform *T_01 (new tf::StampedTransform);    // these are from the old 'TF'
  tf::StampedTransform *T_10 (new tf::StampedTransform);    // they are stil used for pcl_ros::transformPointCloud

  tf::StampedTransform *T_12 (new tf::StampedTransform);
  tf::StampedTransform *T_21 (new tf::StampedTransform);

  tf::StampedTransform *T_02 (new tf::StampedTransform);
  tf::StampedTransform *T_20 (new tf::StampedTransform);

  //tf::StampedTransform *T_01_s (new tf::StampedTransform);coeffs_plane
  //*T_01_s=*T_01;

  //tf2::Transform *T_01_tf2 (new tf2::Transform); // I am not sure if these new tf2 ojects are worth anythings
  //tf2::Transform *T_12_tf2 (new tf2::Transform); // I am sure I wasted hours messing with it though, dum...
  //tf2::Transform *T_10_tf2 (new tf2::Transform);
  //tf2::Transform *T_21_tf2 (new tf2::Transform);

  static tf2_ros::StaticTransformBroadcaster static_broadcaster; // this is the new 'TF2' way to broadcast tfs
  geometry_msgs::TransformStamped *T_01_msg (new geometry_msgs::TransformStamped);
  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";

  geometry_msgs::TransformStamped *T_10_msg (new geometry_msgs::TransformStamped);
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";

  //geometry_msgs::TransformStamped *T_12_msg (new geometry_msgs::TransformStamped);
  //T_12_msg->header.frame_id = "base_link"; T_12_msg->child_frame_id = "T_12";

  //geometry_msgs::TransformStamped *T_21_msg (new geometry_msgs::TransformStamped);
  //T_21_msg->header.frame_id = "base_link"; T_21_msg->child_frame_id = "T_21";

  //geometry_msgs::TransformStamped *T_02_msg (new geometry_msgs::TransformStamped);
  //T_02_msg->header.frame_id = "base_link"; T_02_msg->child_frame_id = "T_02";

  //geometry_msgs::TransformStamped *T_20_msg (new geometry_msgs::TransformStamped);
  //T_20_msg->header.frame_id = "base_link"; T_20_msg->child_frame_id = "T_20";

  pcl::ModelCoefficients::Ptr coeffs_plane (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coeffs_cylinder (new pcl::ModelCoefficients);


  std::cout<<"*************************************************************"<<endl;
  std::cout<<"*************** Processing Pointcloud Data ******************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;

  // Perform RANSAC Segmentation to separate clouds and find part of interest
  segment_cloud(*cloud_lidar,*cloud_part1,*cloud_part2,*cloud_filtered,part1_type,coeffs_plane,coeffs_cylinder);

  
  // Perform ICP Cloud Registration to find location and orientation of part of interest
  register_cloud_icp(*cloud_cad1, *cloud_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, icp_params);

  register_cloud_teaser(*cloud_cad1, *cloud_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, icp_params);


  //std::cout<<"Computing Matrix Inverse"<<std::endl;

  // now move the CAD part to the newly located frame
  pcl_ros::transformPointCloud(*cloud_cad1,*cloud_cad2,*T_01); // this works with 'pcl::PointCloud<pcl::PointXYZ>' and 'tf::Transform'
  std::cout << "Cloud transformed." << std::endl;
  //tf2::doTransform(*cloud_cad1,*cloud_cad2,*T_01_msg); // I have not made this work yet...

  // repeat registration on moved cad model (ICP second pass)
  //register_cloud(*cloud_cad2, *cloud_part1, *T_21, *T_12, *T_21_msg, *T_12_msg,icp_params);

  // now move the CAD part again to the newly located frame
  //pcl_ros::transformPointCloud(*cloud_cad2,*cloud_cad3,*T_12);
  //std::cout << "Cloud transformed again." << std::endl;

  //*T_02=(*T_01)*(*T_12); // multiply the two transforms to get final tf
  //*T_20=T_02->inverse(); // get the inverse of the tf

  //combine_transformation(*T_01,*T_12,*T_20,*T_02,*T_20_msg,*T_02_msg);
  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";

  //T_12_msg->header.frame_id = "base_link"; T_12_msg->child_frame_id = "T_12";
  //T_21_msg->header.frame_id = "base_link"; T_21_msg->child_frame_id = "T_21";

  //T_02_msg->header.frame_id = "base_link"; T_02_msg->child_frame_id = "T_02";
  //T_20_msg->header.frame_id = "base_link"; T_20_msg->child_frame_id = "T_20";

  std::cout << "Final transformation computed and converted to message." <<endl;
  std::cout << "Plane Coefficients" << *coeffs_plane <<endl;

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"*************** Pointcloud Processing Complete *************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"*************** Preparing Visualization *********************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;

  // publish 'markers' to to show the plane and cylinder found with RANSAC
  // instantiate pubs for the plane marker
  ros::Publisher pub_plane = node.advertise<visualization_msgs::Marker>("/marker_plane", 1);
  visualization_msgs::Marker marker_plane; // notice the markers are not pointers
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_plane.ns = "basic_shapes";
  marker_plane.id = 1;
  marker_plane.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker_plane.action = visualization_msgs::Marker::ADD;

  marker_plane.pose.position.x = 0; // set origin
  marker_plane.pose.position.y = 0;
  marker_plane.pose.position.z = 0;

  marker_plane.pose.position.z = -coeffs_plane->values[3];
  //marker_plane.pose.orientation.x = 0;  // set rotation
  marker_plane.pose.orientation.y = 0;
  marker_plane.pose.orientation.z = 0;
  marker_plane.pose.orientation.w = 1;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_plane.scale.x = .001*300;
  marker_plane.scale.y = .001*300;
  marker_plane.scale.z = .001;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_plane.color.r = 1.0f;
  marker_plane.color.g = 0.0f;
  marker_plane.color.b = 0.0f;
  marker_plane.color.a = 0.5;
  // set the header info
  marker_plane.lifetime = ros::Duration();
  marker_plane.header.frame_id = "base_link";
  marker_plane.header.stamp = ros::Time::now();

  //instantiate pubs for the cylinder marker
  ros::Publisher pub_cylinder = node.advertise<visualization_msgs::Marker>("/marker_cylinder", 1);
  visualization_msgs::Marker marker_cylinder; // notice the markers are not pointers
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_cylinder.ns = "basic_shapes";
  marker_cylinder.id = 1;
  marker_cylinder.type = visualization_msgs::Marker::CYLINDER;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker_cylinder.action = visualization_msgs::Marker::ADD;

  marker_cylinder.pose.position.x = 0; // set origin
  marker_cylinder.pose.position.y = 0;
  marker_cylinder.pose.position.z = 0;
  marker_cylinder.pose.orientation.x = 0;  // set rotation
  marker_cylinder.pose.orientation.y = 0;
  marker_cylinder.pose.orientation.z = 0;
  marker_cylinder.pose.orientation.w = 1;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_cylinder.scale.x = .05;
  marker_cylinder.scale.y = .05;
  marker_cylinder.scale.z = .1;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_cylinder.color.r = 1.0f;
  marker_cylinder.color.g = 0.0f;
  marker_cylinder.color.b = 0.0f;
  marker_cylinder.color.a = 0.5;
  // set the header info
  marker_cylinder.lifetime = ros::Duration();
  marker_cylinder.header.frame_id = "base_link";
  marker_cylinder.header.stamp = ros::Time::now();

  //print_tf(*T_02);
  //print_tf(*T_01); // print the info in the TFs for debugging
  //print_tf(*T_10);
  //print_tf(*T_12);
  //print_tf(*T_21);
  //print_tf(*T_02);
  //print_tf(*T_20);

  ros::Publisher pub_lidar = node.advertise<PointCloud> ("/cloud_lidar", 1) ;
  ros::Publisher pub_filtered = node.advertise<PointCloud> ("/cloud_filtered", 1) ;
  ros::Publisher pub_cad1 = node.advertise<PointCloud> ("/cloud_cad1", 1) ;
  ros::Publisher pub_cad2 = node.advertise<PointCloud> ("/cloud_cad2", 1) ;
  ros::Publisher pub_cad3 = node.advertise<PointCloud> ("/cloud_cad3", 1) ;
  ros::Publisher pub_part1 = node.advertise<PointCloud> ("/cloud_part1", 1) ;
  ros::Publisher pub_part2 = node.advertise<PointCloud> ("/cloud_part2", 1) ;

  cloud_lidar->header.frame_id = "base_link";
  cloud_filtered->header.frame_id = "base_link";
  cloud_cad1->header.frame_id = "base_link";
  cloud_cad2->header.frame_id = "base_link";
  cloud_cad3->header.frame_id = "base_link";
  cloud_part1->header.frame_id = "base_link";
  cloud_part2->header.frame_id = "base_link";

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"****************** seam_detection Complete ******************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;

  //publish forever
  while(ros::ok())
  {

      // this is the new 'TF2' way of broadcasting tfs
      T_01_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_msg);
      T_10_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_msg);

      //T_12_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_12_msg);
      //T_21_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_21_msg);

      //T_02_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_02_msg);
      //T_20_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_20_msg);

      pub_lidar.publish(cloud_lidar);
      pub_filtered.publish(cloud_filtered);
      pub_cad1.publish(cloud_cad1);
      pub_cad2.publish(cloud_cad2);
      pub_cad3.publish(cloud_cad3);
      pub_part1.publish(cloud_part1);
      pub_part2.publish(cloud_part2);

      pub_plane.publish(marker_plane);
      pub_cylinder.publish(marker_cylinder);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

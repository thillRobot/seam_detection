/*
This source contains the primary approach and methods for:

"Automated Weld Path Generation Using Random Sample Consensus and Iterative Closest
Point Workpiece Localization" - IDETC2021

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
v1.5 - 03/10/2021 

v1.6 - 12/14/2022 

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
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;

// this function applies a bounding box and a voxel filter to the input cloud
void filter_cloud(PointCloud &cloud_input, PointCloud &cloud_output, double box[], double leaf_size)
{

  //double xmin, xmax, ymin, ymax, zmin, zmax;; // this could be done without these copies
  //xmin=box[0];xmax=box[1];                        // for now they are being used 
  //ymin=box[2];ymax=box[3];
  //zmin=box[4];zmax=box[5];
  //leaf_size=params[6];

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_input,*cloud);

  std::cout<<"BEGINNING CLOUD FILTERING" << std::endl;
  std::cout<<"Before filtering there are "<<cloud->width * cloud->height << " data points in the lidar (source) cloud. "<< std::endl;

  //Apply Bounding Box Filter

  pcl::PassThrough<pcl::PointXYZ> pass;cloud_input,
  pass.setInputCloud(cloud);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(box[0],box[1]);
  pass.filter (*cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(box[2],box[3]);
  pass.filter (*cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits(box[4],box[5]);
  pass.filter (*cloud);

  std::cout<<"Bounding Box Filter Limits: [" <<box[0]<<","<<box[1]<<","<<box[2]<<","<<box[3]<<","<<box[4]<<","<<box[5]<<"]"<< std::endl;
  std::cout<<"After bounding box filter there are "<<cloud->width * cloud->height << " data points in the lidar cloud. "<< std::endl;

  // Apply Voxel Filter 

  if (leaf_size>0)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (leaf_size, leaf_size, leaf_size); // use "001f","001f","0001f" or "none" to set voxel leaf size
    vox.filter (*cloud);
    std::cout<<"After voxel filtering there are "<<cloud->width * cloud->height << " data points in the lidar cloud. "<< std::endl;
  }else
  {
    std::cout<<"No voxel filtering"<< std::endl;
  }

  pcl::copyPointCloud(*cloud,cloud_output);

}

// This function takes the lidar cloud and separates or segments the cloud into different parts
void segment_cloud(PointCloud &cloud_input, PointCloud &cloud_output1, PointCloud &cloud_output2, PointCloud &cloud_output3,PointCloud &cloud_output4, const std::string& part1_type, double norm_dist_wt, double max_iter, double dist_thrsh, double k_srch, double init_norm[])
{

  // instantiate objects needed for segment_cloud function
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  //Point CLoud Datasets - local to this function - could save resources here by not having copies for each step of cascade
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>),
                                    cloud_normals2 (new pcl::PointCloud<pcl::Normal>),
                                    cloud_normals3 (new pcl::PointCloud<pcl::Normal>),
                                    cloud_normals4 (new pcl::PointCloud<pcl::Normal>);

  pcl::PointCloud<PointT>::Ptr cloud_filtered1 (new pcl::PointCloud<PointT>),
                               cloud_filtered2 (new pcl::PointCloud<PointT>),
                               cloud_filtered3 (new pcl::PointCloud<PointT>),
                               cloud_filtered4 (new pcl::PointCloud<PointT>),
                               cloud_filtered5 (new pcl::PointCloud<PointT>);

  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients), 
                              coefficients_plane1 (new pcl::ModelCoefficients),
                              coefficients_plane2 (new pcl::ModelCoefficients), 
                              coefficients_plane3 (new pcl::ModelCoefficients),
                              coefficients_plane4 (new pcl::ModelCoefficients);
  
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices),
                         inliers_plane1 (new pcl::PointIndices), 
                         inliers_plane2 (new pcl::PointIndices), 
                         inliers_plane3 (new pcl::PointIndices),
                         inliers_plane4 (new pcl::PointIndices);
  
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ()),
                               cloud_plane1 (new pcl::PointCloud<PointT> ()), 
                               cloud_plane2 (new pcl::PointCloud<PointT> ()), 
                               cloud_plane3 (new pcl::PointCloud<PointT> ()),
                               cloud_plane4 (new pcl::PointCloud<PointT> ()),  
                               cloud_part1 (new pcl::PointCloud<PointT> ());
  
  // copy the input cloud to begin the segmentation 
  pcl::copyPointCloud(cloud_input,*cloud_filtered1); // the naming here could use work

  std::cout <<"BEGINNING SEGMENTATION" << std::endl;
  std::cout<<"Performing First Segmentaion on " <<cloud_filtered1->width * cloud_filtered1->height << " points"<<std::endl;
  std::cout<<"Searching for plate/table as: SACMODEL_NORMAL_PLANE"<< std::endl; // as a single plane?

 
  //double norm_dist_wt, max_iter, dist_thrsh, k_srch;
  
  //norm_dist_wt=params[0];   // this still needs work, there are few more not here that are used below
  //max_iter=params[1];
  //dist_thrsh=params[2];
  //k_srch=params[3];

   // Estimate point normals before segmentation
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered1);
  ne.setKSearch (k_srch);
  ne.compute (*cloud_normals1);

  // Perform RANSAC Segmentation to find plane of the table first  
  // Instantiate segmentation object for the planar model and set parameters // should these params be exposed ?
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (norm_dist_wt);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (max_iter);
  seg.setDistanceThreshold (dist_thrsh);
  //seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered1);
  seg.setInputNormals (cloud_normals1);

  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane1, *coefficients_plane1);
  //*C_plane=*coefficients_plane1;      // what is this? 
  std::cout << "Plane coefficients: " << *coefficients_plane1 << std::endl;

  // Extract the planar inliers from the input (filtered) cloud - these are points in the plane model
  extract.setInputCloud (cloud_filtered1);
  extract.setIndices (inliers_plane1);
  extract.setNegative (false);
  extract.filter (*cloud_plane1);
  // Remove the planar inliers and proceed with the rest of the points
  extract.setNegative (true);
  extract.filter (*cloud_filtered2); // cloud filtered2 are the outliers of the plane segmentation
  
  //Apply Z Filter - this is a patch 

  //pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud(cloud_filtered2);

  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits(-0.01,0.5);
  //pass.filter (*cloud_filtered2);


  std::cout << "The PointCloud representing the planar component contains: " << cloud_plane1->points.size () << " data points." << std::endl;

  if (part1_type=="round_tube") //part one is a cylinder - this is set in the config file
  {

    std::cout<<"Performing Second Segmentaion"<<std::endl;
    std::cout<<"Searching for round-tube as a cylinder as: SACMODEL_CYLINDER"<<std::endl;

    // Estimate point normals before segmentation
    ne.setInputCloud (cloud_filtered2);
    ne.compute (*cloud_normals2);

    // Create segmentation object for cylinder segmentation and set parameters
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setRadiusLimits (0, 0.1);
    //seg.setMaxIterations (100);
    //seg.setDistanceThreshold (0.05);
    
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //*C_cylinder=*coefficients_cylinder;
    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // copy cylinder inliers to cloud
    extract.setInputCloud (cloud_filtered2);
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


  }else if (part1_type=="square_tube") // part two is a square tube - collection of orthogonal planes
  {

    std::cout<<"Performing Second Segmentation"<<std::endl;
    std::cout<<"Searching for square-tube as: pcl::SACMODEL_PERPENDICULAR_PLANE"<< std::endl; // as a single plane?
    std::cout<<"Before extracting plane 2: " << cloud_filtered2->points.size () << " data points." << std::endl;

    // Estimate point normals
    ne.setInputCloud (cloud_filtered2);
    ne.compute (*cloud_normals2);

    // set parameters and perform segmentation 
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    //seg.setMaxIterations (100);
    //seg.setDistanceThreshold (0.0015);

    // choose an normal vector for the perpendicular plane segmentation
    //seg.setAxis (Eigen::Vector3f (-0.5, 1.0, 0.0)); // this needs to be set in the config file!
    seg.setAxis (Eigen::Vector3f (init_norm[0], init_norm[1], init_norm[2])); // and now it is!
    seg.setEpsAngle (0.1);
    seg.setMaxIterations (1000); // parameter change from those set in config file
    seg.setDistanceThreshold (0.0015); 
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane2, *coefficients_plane2);
    //*C_plane=*coefficients_plane2;

    // copy the model inliers to a cloud 
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_plane2);
    extract.setNegative (false);
    extract.filter (*cloud_plane2);
    extract.setNegative (true);
    extract.filter (*cloud_filtered3);

    std::cout << "Extracted first plane of part 1 with " << cloud_plane2->points.size () << " data points." << std::endl;
    std::cout << "There are " << cloud_filtered3->points.size () << " data points remaining." << std::endl;

    ne.setInputCloud (cloud_filtered3);
    ne.compute (*cloud_normals3);

    // set parameters and perform segmentation 
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    // set the search axis to the cross product of the axis normal to plane 2, this should give an alternate face of the square tube 
    seg.setAxis (Eigen::Vector3f (-coefficients_plane2->values[1],coefficients_plane2->values[0], 0.0)); 
    //seg.setEpsAngle (0.1); 
    //seg.setMaxIterations (1000);
    //seg.setDistanceThreshold (0.0015);
    seg.setInputCloud (cloud_filtered3);
    seg.setInputNormals (cloud_normals3);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane3, *coefficients_plane3);
    //*C_plane=*coefficients_plane3;

    // copy the model inliers to a cloud 
    extract.setInputCloud (cloud_filtered3);
    extract.setIndices (inliers_plane3);
    extract.setNegative (false);
    extract.filter (*cloud_plane3);
    extract.setNegative (true);
    extract.filter (*cloud_filtered4);

    std::cout << "Extracted second plane of part 1 with " << cloud_plane3->points.size () << " data points." << std::endl;
    std::cout << "There are " << cloud_filtered4->points.size () << " data points remaining." << std::endl;

    ne.setInputCloud (cloud_filtered4);
    ne.compute (*cloud_normals4);

    // set parameters and perform segmentation 
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    // set the search axis to the cross product of the axis normal to plane 2, this should give an alternate face of the square tube 
    float A_plane4,B_plane4,C_plane4;
    A_plane4=(coefficients_plane2->values[1])*(coefficients_plane3->values[2])-(coefficients_plane2->values[2])*(coefficients_plane3->values[1]);
    B_plane4=(coefficients_plane2->values[2])*(coefficients_plane3->values[0])-(coefficients_plane2->values[0])*(coefficients_plane3->values[2]);
    C_plane4=(coefficients_plane2->values[0])*(coefficients_plane3->values[1])-(coefficients_plane2->values[1])*(coefficients_plane3->values[0]);   

    seg.setAxis (Eigen::Vector3f ( A_plane4, B_plane4, C_plane4)); 
    //seg.setEpsAngle (0.1); 
    //seg.setMaxIterations (1000);
    //seg.setDistanceThreshold (0.0015);
    seg.setInputCloud (cloud_filtered4);
    seg.setInputNormals (cloud_normals4);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane4, *coefficients_plane4);
    //*C_plane=*coefficients_plane3;

    // copy the model inliers to a cloud 
    extract.setInputCloud (cloud_filtered4);
    extract.setIndices (inliers_plane4);
    extract.setNegative (false);
    extract.filter (*cloud_plane4);
    extract.setNegative (true);
    extract.filter (*cloud_filtered5);

    std::cout << "Extracted third plane of part 1 with " << cloud_plane4->points.size () << " data points." << std::endl;
    std::cout << "There are " << cloud_filtered5->points.size () << " data points remaining." << std::endl;

    *cloud_part1=(*cloud_plane2)+(*cloud_plane3); // concatenate clouds to work the
    *cloud_part1=(*cloud_part1)+(*cloud_plane4);  // perform the concatenation in two steps becauase one line did not work
    
    if (cloud_plane2->points.empty ())
      std::cout << "Cannot find the plane component of cloud." << std::endl;
    else
    {
      std::cout << "PointCloud representing the plane component: " << cloud_plane2->points.size () << " data points." << std::endl;
      std::cout << "PointCloud representing the plane component: " << cloud_plane3->points.size () << " data points." << std::endl;
      std::cout << "PointCloud representing the plane component: " << cloud_plane4->points.size () << " data points." << std::endl;
      std::cout << "PointCloud representing the part component: " << cloud_part1->points.size () << " data points." << std::endl;
      std::cout << "Plane3 Coefs: " << coefficients_plane2->values[0] << std::endl 
                                    << coefficients_plane2->values[1] << std::endl 
                                    << coefficients_plane2->values[2] << std::endl 
                                    << coefficients_plane2->values[3] << std::endl;
      //writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }
    
    // Copy clouds to the function outputs
    pcl::copyPointCloud(*cloud_part1,cloud_output1);  // use second segmentation
    pcl::copyPointCloud(*cloud_plane1,cloud_output2);
    pcl::copyPointCloud(*cloud_filtered2,cloud_output3);
    pcl::copyPointCloud(*cloud_filtered5,cloud_output4);

  }else if (part1_type=="generic")
  {
    std::cout<<"Skipping Second Segmentation"<<std::endl;
    std::cout<<"Proceeding with filtered outliers of first segmentation"<<std::endl;

    pcl::copyPointCloud(*cloud_filtered2,cloud_output1); // ignore second segmentation and copy to output 
  }

  std::cerr << "END OF SEGMENT_CLOUD FUNCTION" << std::endl;

}

// This function REGISTER_CLOUD_ICP finds the transform between two pointclouds using PCL::IterativeClosestPoint
void register_cloud_icp(PointCloud &cloud_target, PointCloud &cloud_source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double max_corr_dist, double max_iter, double trns_epsl, double ecld_fitn_epsl, double e_results[],double c_offset[])
{
 
  // make a copy of the CAD(target) cloud called 'cloud_A' 
  PointCloud::Ptr cloud_A (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_target,*cloud_A);
  // make a copy of the LiDAR(source) cloud called 'cloud_B'
  PointCloud::Ptr cloud_B (new PointCloud);       //use this as the working copy of the source cloud
  pcl::copyPointCloud(cloud_source,*cloud_B);

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

  icp.setInputTarget(cloud_A); // target (fixed) cloud
  icp.setInputSource(cloud_B);  // source (moved during ICP) cloud
  icp.align(Final); // do ICP

  T_result=icp.getFinalTransformation(); // get the resutls of ICP
  T_inverse=T_result.inverse();

  std::cout << "ICP COMPLETED" << std::endl;
  std::cout << "max iterations:" << icp.getMaximumIterations() << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << "transformation: " << std::endl<< T_result << std::endl;
  std::cout << "inverse: "<< std::endl << T_inverse << std::endl;
  //std::cout << "ICP Algorithm Information: " << std::endl;
  //std::cout <<  icp.getSearchMethodTarget() << std::endl;

  // This part seems very over bloated !!! 
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

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"********************* Analyzing Results *********************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;
  
  double roll, pitch, yaw;
  R_inverse.getRPY(roll, pitch, yaw);

  std::cout<<"Calibration Translation:  ["<<c_offset[0]<<","
                                          <<c_offset[1]<<","
                                          <<c_offset[2]<<"]"<<std::endl;
  std::cout<<"Calibration Rotation:     ["<<c_offset[3]<<","
                                          <<c_offset[4]<<","
                                          <<c_offset[5]<<"]"<<std::endl;


  std::cout<<"Expected Translation: ["<<e_results[0]<<","
                                      <<e_results[1]<<","
                                      <<e_results[2]<<"]"<<std::endl;
  std::cout<<"Measured Translation: ["<<T_inverse(0,3)<<","
                                      <<T_inverse(1,3)<<","
                                      <<T_inverse(2,3)<<"]"<<std::endl;
  std::cout<<"Difference Translation: ["<<e_results[0]-T_inverse(0,3)<<","
                                      <<e_results[1]-T_inverse(1,3)<<","
                                      <<e_results[2]-T_inverse(2,3)<<"]"<<std::endl;
 std::cout<<"Corrected Translation: ["<<T_inverse(0,3)+c_offset[0]<<","
                                      <<T_inverse(1,3)+c_offset[1]<<","
                                      <<T_inverse(2,3)+c_offset[2]<<"]"<<std::endl;
 std::cout<<"Corr Diff Translation: ["<<e_results[0]-T_inverse(0,3)+c_offset[0]<<","
                                      <<e_results[1]-T_inverse(1,3)+c_offset[1]<<","
                                      <<e_results[2]-T_inverse(2,3)+c_offset[2]<<"]"<<std::endl;

    
  std::cout<<"Expected Rotation: [" <<e_results[3]<<","
                                    <<e_results[4]<<","
                                    <<e_results[5]<<"]"<<std::endl;
  std::cout<<"Measured Rotation: [" <<roll
                                    <<","<<pitch
                                    <<","<<yaw<<"]"<<std::endl; 
  std::cout<<"Difference Rotation: ["<<e_results[3]-roll
                                    <<","<<e_results[4]-pitch
                                    <<","<<e_results[5]-yaw<<"]"<<std::endl; 
  std::cout<<"Corrected Rotation: ["<<roll+c_offset[3]<<","
                                      <<pitch+c_offset[4]<<","
                                      <<yaw+c_offset[5]<<"]"<<std::endl;
  std::cout<<"Corr Diff Rotation: ["<<e_results[3]-roll+c_offset[3]<<","
                                    <<e_results[3]-pitch+c_offset[4]<<","
                                    <<e_results[3]-yaw+c_offset[5]<<"]"<<std::endl;


  std::cout << "END OF REGISTER_CLOUD_ICP FUNCTION" << std::endl;
}


// This function REGISTER_CLOUD_TEASER finds the transform between two pointclouds, based on examples/teaser_cpp_ply.cc
void register_cloud_teaser(PointCloud &cloud_target, PointCloud &cloud_source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double tparams[])
{
 
  // get size of inputs clouds
  int Ns = cloud_source.size();
  int Nt = cloud_target.size();
  int P = 50; //number to print
  int M = -1; //number of matches
  std::cout <<"BEGINNING REGISTER_CLOUD_TEASER"<< std::endl;
  std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;

  // pointers to the input clouds, possibly not needed
  //pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  //pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // instantiate teaser pointclouds
  //teaser::PointCloud src_cloud;
  //teaser::PointCloud tgt_cloud;
  
  // Convert the point clouds to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, Ns);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, Nt);
  
  for (size_t i = 0; i < Ns; ++i) {
    src.col(i) << cloud_source[i].x, cloud_source[i].y, cloud_source[i].z;
  }  
  for (size_t i = 0; i < Nt; ++i) {
    tgt.col(i) << cloud_target[i].x, cloud_target[i].y, cloud_target[i].z;
  }
  
  // Convert to homogeneous coordinates
  Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
  src_h.resize(4, src.cols());
  src_h.topRows(3) = src;
  src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(Ns);
  
  Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h;
  tgt_h.resize(4, tgt.cols());
  tgt_h.topRows(3) = tgt;
  tgt_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(Nt);

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
  std::cout << "=====================================" << std::endl;
  std::cout << "          TEASER++ Results           " << std::endl;
  std::cout << "=====================================" << std::endl;
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
  
  std::cout<<"TEASER debug0"<<endl;
  
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


/*
// This function REGISTER_CLOUD finds the transform between two pointclouds using and was written primarily by chatGPT
void register_cloud_teaser_gpt(PointCloud &cloud_target, PointCloud &cloud_source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double params[])
{

  // Convert the point clouds to teaser::PointCloud
  teaser::PointCloud teaserCloud1, teaserCloud2;
  //teaserCloud1.setInputCloud(cloud_target);
  //teaserCloud2.setInputCloud(cloud_source);




  
  // Create a registration object and set the parameters
  teaser::Registration registration;
  
  
  registration.setInputSource(cloud_target);
  registration.setInputTarget(cloud_source);

  // Register the point clouds
  registration.align();

  // Get the transformation matrix
  Eigen::Matrix4f transformation = registration.getFinalTransformation();

  // Print the transformation matrix
  std::cout << "Transformation matrix: " << std::endl << transformation << std::endl;
  
}
*/

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
/*
// this function prints the info in a TF to the console
void analyze_results(tf::Transform &tf_in,double e_results[],double c_offset[])
{
  std::cout<<"*************************************************************"<<endl;
  std::cout<<"********************* Analyzing Results *********************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;
  
  std::cout<<"Calibration Translation:  ["<<c_offset[0]<<","
                                          <<c_offset[1]<<","
                                          <<c_offset[2]<<"]"<<std::endl;
  std::cout<<"Calibration Rotation:     ["<<c_offset[3]<<","
                                          <<c_offset[4]<<","
                                          <<c_offset[5]<<"]"<<std::endl;


  std::cout<<"[Expected,Translation: ["<<e_results[0]<<","
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
*/

/*
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
*/

int main(int argc, char** argv)
{

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"******************** Seam Detection Teaser v1.6 ********************"<<endl;
  std::cout<<"*************************************************************"<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"**************** Loading Configuration File ****************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;

  // there is only one cmd line arg and it is the name of the config file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // parameters that contain strings  
  std::string lidar_src_path, cad_ref_path, lidar_src_file, cad_ref_file, part1_type;

  node.getParam("lidar_src_file", lidar_src_file);
  lidar_src_path=packagepath+'/'+lidar_src_file;

  node.getParam("cad_ref_file", cad_ref_file);
  cad_ref_path=packagepath+'/'+cad_ref_file;

  node.getParam("part1_type",  part1_type);

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

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"******************* Preparing Pointcloud Data ***************"<<endl;
  std::cout<<"*************************************************************"<<endl;

  
  // instantiate some clouds
  PointCloud::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZ>); // target cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); // target cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>); // target cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>); // target cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_cad1 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud
  PointCloud::Ptr cloud_cad2 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud intermediate
  PointCloud::Ptr cloud_cad3 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud final
  PointCloud::Ptr cloud_part1 (new pcl::PointCloud<pcl::PointXYZ>); // cylinder cloud // input to ICP
  PointCloud::Ptr cloud_part2 (new pcl::PointCloud<pcl::PointXYZ>); // plane cloud

  std::cout<<"Debug1"<<endl;


  // load the clouds from file (.pcd)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_src_path, *cloud_lidar) == -1)
  {
      std::cout<<"Couldn't read image file:"<<lidar_src_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< lidar_src_path <<std::endl<<
      cloud_lidar->width * cloud_lidar->height << " Data points from "<< lidar_src_path << std::endl;

  // load the cloud from CAD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (cad_ref_path, *cloud_cad1) == -1)
  {
      std::cout<<"Couldn't read image file:"<<cad_ref_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< cad_ref_path <<std::endl<<
      cloud_cad1->width * cloud_cad1->height << " Data points from "<< cad_ref_path <<std::endl<<std::endl;

  std::cout<<"Debug2"<<endl;

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

  //geometry_msgs::TransformStamped *T_12_msg (new geometry_msgs::TransformStamped); // these were needed to repeat entire approach
  //T_12_msg->header.frame_id = "base_link"; T_12_msg->child_frame_id = "T_12";

  //geometry_msgs::TransformStamped *T_21_msg (new geometry_msgs::TransformStamped);
  //T_21_msg->header.frame_id = "base_link"; T_21_msg->child_frame_id = "T_21";

  //geometry_msgs::TransformStamped *T_02_msg (new geometry_msgs::TransformStamped);
  //T_02_msg->header.frame_id = "base_link"; T_02_msg->child_frame_id = "T_02";

  //geometry_msgs::TransformStamped *T_20_msg (new geometry_msgs::TransformStamped);
  //T_20_msg->header.frame_id = "base_link"; T_20_msg->child_frame_id = "T_20";

  //pcl::ModelCoefficients::Ptr coeffs_plane (new pcl::ModelCoefficients); // used for markers at one point
  //pcl::ModelCoefficients::Ptr coeffs_cylinder (new pcl::ModelCoefficients);


  std::cout<<"*************************************************************"<<endl;
  std::cout<<"*************** Processing Pointcloud Data ******************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;


  // Filter the LiDAR cloud with a bounding box and a voxel (downsampling)
  filter_cloud(*cloud_lidar,*cloud_filtered, filter_box, voxel_leaf_size); 

  // Perform RANSAC Segmentation to separate clouds and find part of interest
  segment_cloud(*cloud_filtered,*cloud_part1,*cloud_part2,*cloud_filtered2,*cloud_filtered3, part1_type, ransac_norm_dist_wt, ransac_max_iter, ransac_dist_thrsh, ransac_k_srch, ransac_init_norm);

  // Perform ICP Cloud Registration to find location and orientation of part of interest
  //register_cloud_icp(*cloud_cad1, *cloud_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl,expected_results,calibration_offset);

  int N_cor=100;

  EigenCor cor_src_pts, cor_tgt_pts;

  // Perform TEASER++ cloud registration
  double teaser_params[3]={1,2,3}; // temporary place holder 
  register_cloud_teaser(*cloud_cad1,*cloud_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);

  //register_cloud_teaser(*cloud_cad1, *cloud_filtered,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);


  // now align the CAD part to using the resulting transformation
  pcl_ros::transformPointCloud(*cloud_cad1,*cloud_cad2,*T_01); // this works with 'pcl::PointCloud<pcl::PointXYZ>' and 'tf::Transform'
  
  //analyze_results(*T_01, expected_results,calibration_offset);

  std::cout << "Cloud aligned using resulting transformation." << std::endl;
  //tf2::doTransform(*cloud_cad1,*cloud_cad2,*T_01_msg); // I have not made this work yet...

  // repeat registration on aligned cad model (ICP second pass)

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
  //std::cout << "Plane Coefficients" << *coeffs_plane <<endl;

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"*************** Pointcloud Processing Complete *************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;

  std::cout<<"*************************************************************"<<endl;
  std::cout<<"*************** Preparing Visualization *********************"<<endl;
  std::cout<<"*************************************************************"<<endl<<endl;

  /*
  // publish 'markers' to to show the plane and cylinder found with RANSAC - disabled for now
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
  */

  //print_tf(*T_02);
  //print_tf(*T_01); // print the info in the TFs for debugging
  //print_tf(*T_10);
  //print_tf(*T_12);
  //print_tf(*T_21);
  //print_tf(*T_02);
  //print_tf(*T_20);

  ros::Publisher pub_lidar = node.advertise<PointCloud> ("/cloud_lidar", 1) ;
  ros::Publisher pub_filtered = node.advertise<PointCloud> ("/cloud_filtered", 1) ;
  ros::Publisher pub_filtered2 = node.advertise<PointCloud> ("/cloud_filtered2", 1) ;
  ros::Publisher pub_filtered3 = node.advertise<PointCloud> ("/cloud_filtered3", 1) ;
  ros::Publisher pub_cad1 = node.advertise<PointCloud> ("/cloud_cad1", 1) ;
  ros::Publisher pub_cad2 = node.advertise<PointCloud> ("/cloud_cad2", 1) ;
  ros::Publisher pub_cad3 = node.advertise<PointCloud> ("/cloud_cad3", 1) ;
  ros::Publisher pub_part1 = node.advertise<PointCloud> ("/cloud_part1", 1) ;
  ros::Publisher pub_part2 = node.advertise<PointCloud> ("/cloud_part2", 1) ;

  cloud_lidar->header.frame_id = "base_link";
  cloud_filtered->header.frame_id = "base_link";
  cloud_filtered2->header.frame_id = "base_link";
  cloud_filtered3->header.frame_id = "base_link";
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
      pub_filtered2.publish(cloud_filtered2);
      pub_filtered3.publish(cloud_filtered3);
      pub_cad1.publish(cloud_cad1);
      pub_cad2.publish(cloud_cad2);
      pub_cad3.publish(cloud_cad3);
      pub_part1.publish(cloud_part1);
      pub_part2.publish(cloud_part2);
      //pub_plane.publish(marker_plane);
      //pub_cylinder.publish(marker_cylinder);

      /*
      pub_pointslines.publish(points_src);
      pub_pointslines.publish(points_tgt);
      pub_pointslines.publish(line_strip);
      pub_pointslines.publish(line_list);
      */

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}



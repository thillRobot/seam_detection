/*
This source contains the primary approach and methods for:

"Automated Weld Path Generation Using Random Sample Consensus and Iterative Closest
Point Workpiece Localization" - IDETC2022

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
updated - 03/08/2023 

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
void filter_cloud(PointCloud &input, PointCloud &output, double box[], double leaf_size)
{

  //double xmin, xmax, ymin, ymax, zmin, zmax;; // this could be done without these copies
  //xmin=box[0];xmax=box[1];                        // for now they are being used 
  //ymin=box[2];ymax=box[3];
  //zmin=box[4];zmax=box[5];
  //leaf_size=params[6];

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(input,*cloud);

  std::cout<<"BEGINNING CLOUD FILTERING" << std::endl;
  std::cout<<"Before filtering there are "<<cloud->width * cloud->height << " data points in the lidar (source) cloud. "<< std::endl;

  //Apply Bounding Box Filter

  pcl::PassThrough<pcl::PointXYZ> pass; //cloud_input,
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

  pcl::copyPointCloud(*cloud,output);

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

  pcl::PointCloud<PointT>::Ptr cloud_source_filtered1 (new pcl::PointCloud<PointT>),
                               cloud_source_filtered2 (new pcl::PointCloud<PointT>),
                               cloud_source_filtered3 (new pcl::PointCloud<PointT>),
                               cloud_source_filtered4 (new pcl::PointCloud<PointT>),
                               cloud_source_filtered5 (new pcl::PointCloud<PointT>);

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
                               cloud_source_part1 (new pcl::PointCloud<PointT> ());
  
  // copy the input cloud to begin the segmentation 
  pcl::copyPointCloud(cloud_input,*cloud_source_filtered1); // the naming here could use work

  std::cout <<"BEGINNING SEGMENTATION" << std::endl;
  std::cout<<"Performing First Segmentaion on " <<cloud_source_filtered1->width * cloud_source_filtered1->height << " points"<<std::endl;
  std::cout<<"Searching for plate/table as: SACMODEL_NORMAL_PLANE"<< std::endl; // as a single plane?

 
  //double norm_dist_wt, max_iter, dist_thrsh, k_srch;
  
  //norm_dist_wt=params[0];   // this still needs work, there are few more not here that are used below
  //max_iter=params[1];
  //dist_thrsh=params[2];
  //k_srch=params[3];

   // Estimate point normals before segmentation
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_source_filtered1);
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
  seg.setInputCloud (cloud_source_filtered1);
  seg.setInputNormals (cloud_normals1);

  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane1, *coefficients_plane1);
  //*C_plane=*coefficients_plane1;      // what is this? 
  std::cout << "Plane coefficients: " << *coefficients_plane1 << std::endl;

  // Extract the planar inliers from the input (filtered) cloud - these are points in the plane model
  extract.setInputCloud (cloud_source_filtered1);
  extract.setIndices (inliers_plane1);
  extract.setNegative (false);
  extract.filter (*cloud_plane1);
  // Remove the planar inliers and proceed with the rest of the points
  extract.setNegative (true);
  extract.filter (*cloud_source_filtered2); // cloud filtered2 are the outliers of the plane segmentation
  
  //Apply Z Filter - this is a patch 

  //pcl::PassThrough<pcl::PointXYZ> pass;
  //pass.setInputCloud(cloud_source_filtered2);

  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits(0.0,0.5);
  //  pass.filter (*cloud_source_filtered2);


  std::cout << "The PointCloud representing the planar component contains: " << cloud_plane1->points.size () << " data points." << std::endl;

  if (part1_type=="round_tube") //part one is a cylinder - this is set in the config file
  {

    std::cout<<"Performing Second Segmentaion"<<std::endl;
    std::cout<<"Searching for round-tube as a cylinder as: SACMODEL_CYLINDER"<<std::endl;

    // Estimate point normals before segmentation
    ne.setInputCloud (cloud_source_filtered2);
    ne.compute (*cloud_normals2);

    // Create segmentation object for cylinder segmentation and set parameters
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setRadiusLimits (0, 0.1);
    //seg.setMaxIterations (100);
    //seg.setDistanceThreshold (0.05);
    
    seg.setInputCloud (cloud_source_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //*C_cylinder=*coefficients_cylinder;
    std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // copy cylinder inliers to cloud
    extract.setInputCloud (cloud_source_filtered2);
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
    std::cout<<"Before extracting plane 2: " << cloud_source_filtered2->points.size () << " data points." << std::endl;

    // Estimate point normals
    ne.setInputCloud (cloud_source_filtered2);
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
    seg.setInputCloud (cloud_source_filtered2);
    seg.setInputNormals (cloud_normals2);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane2, *coefficients_plane2);
    //*C_plane=*coefficients_plane2;

    // copy the model inliers to a cloud 
    extract.setInputCloud (cloud_source_filtered2);
    extract.setIndices (inliers_plane2);
    extract.setNegative (false);
    extract.filter (*cloud_plane2);
    extract.setNegative (true);
    extract.filter (*cloud_source_filtered3);

    std::cout << "Extracted first plane of part 1 with " << cloud_plane2->points.size () << " data points." << std::endl;
    std::cout << "There are " << cloud_source_filtered3->points.size () << " data points remaining." << std::endl;

    ne.setInputCloud (cloud_source_filtered3);
    ne.compute (*cloud_normals3);

    // set parameters and perform segmentation 
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    // set the search axis to the cross product of the axis normal to plane 2, this should give an alternate face of the square tube 
    seg.setAxis (Eigen::Vector3f (-coefficients_plane2->values[1],coefficients_plane2->values[0], 0.0)); 
    //seg.setEpsAngle (0.1); 
    //seg.setMaxIterations (1000);
    //seg.setDistanceThreshold (0.0015);
    seg.setInputCloud (cloud_source_filtered3);
    seg.setInputNormals (cloud_normals3);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane3, *coefficients_plane3);
    //*C_plane=*coefficients_plane3;

    // copy the model inliers to a cloud 
    extract.setInputCloud (cloud_source_filtered3);
    extract.setIndices (inliers_plane3);
    extract.setNegative (false);
    extract.filter (*cloud_plane3);
    extract.setNegative (true);
    extract.filter (*cloud_source_filtered4);

    std::cout << "Extracted second plane of part 1 with " << cloud_plane3->points.size () << " data points." << std::endl;
    std::cout << "There are " << cloud_source_filtered4->points.size () << " data points remaining." << std::endl;

    ne.setInputCloud (cloud_source_filtered4);
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
    seg.setInputCloud (cloud_source_filtered4);
    seg.setInputNormals (cloud_normals4);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane4, *coefficients_plane4);
    //*C_plane=*coefficients_plane3;

    // copy the model inliers to a cloud 
    extract.setInputCloud (cloud_source_filtered4);
    extract.setIndices (inliers_plane4);
    extract.setNegative (false);
    extract.filter (*cloud_plane4);
    extract.setNegative (true);
    extract.filter (*cloud_source_filtered5);

    std::cout << "Extracted third plane of part 1 with " << cloud_plane4->points.size () << " data points." << std::endl;
    std::cout << "There are " << cloud_source_filtered5->points.size () << " data points remaining." << std::endl;

    *cloud_source_part1=(*cloud_plane2)+(*cloud_plane3); // concatenate clouds to work the
    *cloud_source_part1=(*cloud_source_part1)+(*cloud_plane4);  // perform the concatenation in two steps becauase one line did not work
    
    if (cloud_plane2->points.empty ())
      std::cout << "Cannot find the plane component of cloud." << std::endl;
    else
    {
      std::cout << "PointCloud representing the plane component: " << cloud_plane2->points.size () << " data points." << std::endl;
      std::cout << "PointCloud representing the plane component: " << cloud_plane3->points.size () << " data points." << std::endl;
      std::cout << "PointCloud representing the plane component: " << cloud_plane4->points.size () << " data points." << std::endl;
      std::cout << "PointCloud representing the part component: " << cloud_source_part1->points.size () << " data points." << std::endl;
      std::cout << "Plane3 Coefs: " << coefficients_plane2->values[0] << std::endl 
                                    << coefficients_plane2->values[1] << std::endl 
                                    << coefficients_plane2->values[2] << std::endl 
                                    << coefficients_plane2->values[3] << std::endl;
      //writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }
    
    // Copy clouds to the function outputs
    pcl::copyPointCloud(*cloud_source_part1,cloud_output1);  // use second segmentation
    pcl::copyPointCloud(*cloud_plane1,cloud_output2);
    pcl::copyPointCloud(*cloud_source_filtered2,cloud_output3);
    pcl::copyPointCloud(*cloud_source_filtered5,cloud_output4);

  }else if (part1_type=="generic")
  {
    std::cout<<"Skipping Second Segmentation"<<std::endl;
    std::cout<<"Proceeding with filtered outliers of first segmentation"<<std::endl;

    pcl::copyPointCloud(*cloud_source_filtered2,cloud_output1); // ignore second segmentation and copy to output 
  }

  std::cerr << "SEGMENT_CLOUD Completed" << std::endl;

}

// This function REGISTER_CLOUD_ICP finds the transform between two pointclouds using PCL::IterativeClosestPoint
void register_cloud_icp(PointCloud &source, PointCloud &target, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double max_corr_dist, double max_iter, double trns_epsl, double ecld_fitn_epsl, double e_results[],double c_offset[])
{
 
  // get size of inputs clouds
  int Ns = source.size();
  int Nt = target.size();
 
  
  // make a copy of the LiDAR(source) cloud 
  PointCloud::Ptr src (new PointCloud);       //use this as the working copy of the source cloud
  pcl::copyPointCloud(source,*src);
  // make a copy of the CAD(target) cloud c
  PointCloud::Ptr tgt (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(target,*tgt);


  std::cout<<"Beginning REGISTER_CLOUD_ICP" << std::endl;
  std::cout <<"Processing "<<Ns<<" source points and" << Nt << " target points" << std::endl ;
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

  icp.setInputSource(src); // source (moved during ICP) cloud
  icp.setInputTarget(tgt); // target (fixed) cloud
  
  icp.align(Final); // do ICP

  T_result=icp.getFinalTransformation(); // get the resutls of ICP
  T_inverse=T_result.inverse();

  std::cout << "ICP Completed" << std::endl;
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

  std::cout << "REGISTER_CLOUD_ICP Completed" << std::endl;
}

// This function REGISTER_CLOUD_TEASER finds the transform between two pointclouds, based on examples/teaser_cpp_ply.cc
void register_cloud_teaser(PointCloud &source, PointCloud &target, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double tparams[])
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

  std::cout << "REGISTER_CLOUD_TEASER Completed" << std::endl;

}


// This function REGISTER_CLOUD_TEASER finds the transform between two pointclouds, based on examples/teaser_cpp_ply.cc
void register_cloud_teaser_fpfh(PointCloud &source, PointCloud &target, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA, double tparams[])
{
 
  // get size of inputs clouds
  int Ns = source.size();
  int Nt = target.size();
  int P = 50; //number to print
  int M = -1; //number of matches
  std::cout <<"Beginning REGISTER_CLOUD_TEASER_FPFH"<< std::endl;
  std::cout <<"Processing "<< Ns << " source points and " <<Nt<<" target points" << std::endl ;

  // instantiate teaser pointclouds
  teaser::PointCloud tgt;
  teaser::PointCloud src;
  

  for (size_t i = 0; i < Ns; ++i) {
    src.push_back({static_cast<float>(source[i].x), static_cast<float>(source[i].y), static_cast<float>(source[i].z)});
  }
  for (size_t i = 0; i < Nt; ++i) {
    tgt.push_back({static_cast<float>(target[i].x), static_cast<float>(target[i].y), static_cast<float>(target[i].z)});
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

  std::cout << "REGISTER_CLOUD_TEASER_FPFH Completed" << std::endl;

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

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"===================================== ==============================="<<endl;
  std::cout<<"                     Seam Detection Teaser v1.6                     "<<endl;
  std::cout<<"===================================================================="<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                      Loading Configuration File                    "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // there is only one cmd line arg and it is the name of the config file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // parameters that contain strings  
  std::string source_path, target_path, source_file, target_file, part1_type;

  node.getParam("source_file", source_file);
  source_path=packagepath+'/'+source_file;

  node.getParam("target_file", target_file);
  target_path=packagepath+'/'+target_file;

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

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     Preparing Pointcloud Data                      "<<endl;
  std::cout<<"===================================================================="<<endl;

  
  // instantiate some clouds
  PointCloud::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_source_filtered (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_source_filtered2 (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_source_filtered3 (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_target1 (new pcl::PointCloud<pcl::PointXYZ>);  // target cloud
  PointCloud::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZ>);  // target cloud aligned by resulting T
  //PointCloud::Ptr cloud_target3 (new pcl::PointCloud<pcl::PointXYZ>);  // target cloud final
  PointCloud::Ptr cloud_source_part1 (new pcl::PointCloud<pcl::PointXYZ>); // cylinder cloud // input to ICP
  PointCloud::Ptr cloud_source_part2 (new pcl::PointCloud<pcl::PointXYZ>); // plane cloud

  std::cout<<"Debug1"<<endl;


  // load the clouds from file (.pcd)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (source_path, *cloud_source) == -1)
  {
      std::cout<<"Couldn't read image file:"<<source_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< source_path <<std::endl<<
      cloud_source->width * cloud_source->height << " Data points from "<< source_path << std::endl;

  // load the cloud from CAD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_path, *cloud_target1) == -1)
  {
      std::cout<<"Couldn't read image file:"<<target_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< target_path <<std::endl<<
      cloud_target1->width * cloud_target1->height << " Data points from "<< target_path <<std::endl<<std::endl;

  std::cout<<"Debug2"<<endl;

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


  // Filter the LiDAR cloud with a bounding box and a voxel (downsampling)
  filter_cloud(*cloud_source,*cloud_source_filtered, filter_box, voxel_leaf_size); 

  // Perform RANSAC Segmentation to separate clouds and find part of interest
  segment_cloud(*cloud_source_filtered,*cloud_source_part1,*cloud_source_part2,*cloud_source_filtered2,*cloud_source_filtered3, part1_type, ransac_norm_dist_wt, ransac_max_iter, ransac_dist_thrsh, ransac_k_srch, ransac_init_norm);

  // Perform ICP Cloud Registration to find location and orientation of part of interest
  register_cloud_icp(*cloud_source_part1,*cloud_target1, *T_10, *T_01, *T_10_msg, *T_01_msg, icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl,expected_results,calibration_offset);

  int N_cor=100;

  EigenCor cor_src_pts, cor_tgt_pts;

  // Perform TEASER++ cloud registration
  double teaser_params[3]={1,2,3}; // temporary place holder 
  //register_cloud_teaser(*cloud_source_part1,*cloud_target1,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);
  
  //register_cloud_teaser_fpfh(*cloud_source_part1,*cloud_target1,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);


  // now align the CAD part to using the resulting transformation
  pcl_ros::transformPointCloud(*cloud_target1,*cloud_aligned,*T_01); // this works with 'pcl::PointCloud<pcl::PointXYZ>' and 'tf::Transform'
  std::cout << "Cloud aligned using resulting transformation." << std::endl;


  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Analzing Results                                "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  analyze_results(*T_10, expected_results);
  analyze_results(*T_01, expected_results);

  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   Pointcloud Processing Complete                   "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   Preparing Visualization                          "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  ros::Publisher pub_source = node.advertise<PointCloud> ("/cloud_source", 1) ;
  ros::Publisher pub_filtered = node.advertise<PointCloud> ("/cloud_source_filtered", 1) ;
  ros::Publisher pub_filtered2 = node.advertise<PointCloud> ("/cloud_source_filtered2", 1) ;
  ros::Publisher pub_filtered3 = node.advertise<PointCloud> ("/cloud_source_filtered3", 1) ;
  ros::Publisher pub_target1 = node.advertise<PointCloud> ("/cloud_target1", 1) ;
  ros::Publisher pub_target2 = node.advertise<PointCloud> ("/cloud_aligned", 1) ;
  ros::Publisher pub_target3 = node.advertise<PointCloud> ("/cloud_target3", 1) ;
  ros::Publisher pub_part1 = node.advertise<PointCloud> ("/cloud_source_part1", 1) ;
  ros::Publisher pub_part2 = node.advertise<PointCloud> ("/cloud_source_part2", 1) ;

  cloud_source->header.frame_id = "base_link";
  cloud_source_filtered->header.frame_id = "base_link";
  cloud_source_filtered2->header.frame_id = "base_link";
  cloud_source_filtered3->header.frame_id = "base_link";
  cloud_target1->header.frame_id = "base_link";
  cloud_aligned->header.frame_id = "base_link";
  //cloud_target3->header.frame_id = "base_link";
  cloud_source_part1->header.frame_id = "base_link";
  cloud_source_part2->header.frame_id = "base_link";

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                        seam_detection Complete                     "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  //publish forever
  while(ros::ok())
  {

      // this is the new 'TF2' way of broadcasting tfs
      T_01_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_msg);
      T_10_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_msg);

      pub_source.publish(cloud_source);
      pub_filtered.publish(cloud_source_filtered);
      pub_filtered2.publish(cloud_source_filtered2);
      pub_filtered3.publish(cloud_source_filtered3);
      pub_part1.publish(cloud_source_part1);
      pub_part2.publish(cloud_source_part2);
      pub_target1.publish(cloud_target1);
      pub_target2.publish(cloud_aligned);
    //  pub_target3.publish(cloud_target3);
      
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}



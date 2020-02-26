/*
RANSAC/Segementation based multiple plane detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University

Taken from PCL sample code - 02/14/2018
Updated - 02/17/2018

Revisited 02/22/2020

Added 'cylinder segmentation' - Boom!

Next we want to add a 'second pass' to refine the results
Before doing this I would like to add a marker in rviz of search spaces (boxes), maybe later...

Robotics Research Group - Mechanical Engineering
*/

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf/LinearMath/Matrix3x3.h>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl_ros/transforms.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void segment_cloud(PointCloud &cloud_input, PointCloud &cloud_output1, PointCloud &cloud_output2)
{
  // make a copy of the lidar cloud called 'cloud'
  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_input,*cloud);

  // XYZ Box Filter cloud before segementation
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(-0.0,0.5);
  pass.filter (*cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(-0.0,0.5);
  pass.filter (*cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits(-0.0,0.5);
  pass.filter (*cloud);

  std::cout<<"After pre-filtering there are "<<cloud->width * cloud->height << " data points in the lidar cloud. "<< std::endl;

  //pcl::copyPointCloud(*cloud,cloud_A); // save input to RANSAC algorithm
  // Perform RANSAC Segmentation to find cylinder
  // instantiate all objects needed
  pcl::PCDReader reader;
  //pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets - local to this function
  //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
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
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);

  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ())
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  }

  pcl::copyPointCloud(*cloud_cylinder,cloud_output1);
  pcl::copyPointCloud(*cloud_plane,cloud_output2);
  std::cerr << "END OF SEGMENT_CLOUD FUNCTION" << std::endl;

}


// This function REGISTER_CLOUD finds the transform between two pointclouds
void register_cloud(PointCloud &cloud_target, PointCloud &cloud_source, tf::Transform &result)
{

  // make 2 copy of the lidar cloud called 'cloud_A' and 'cloud_B'
  PointCloud::Ptr cloud_A (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_target,*cloud_A);
  // make a copy of the lidar cloud called 'cloud'
  PointCloud::Ptr cloud_B (new PointCloud);       //use this as the working copy of the source cloud
  pcl::copyPointCloud(cloud_source,*cloud_B);

  // XYZ Box Filter cloud before segementation
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_A);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(-0.0,0.5);
  pass.filter (*cloud_A);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(-0.0,0.5);
  pass.filter (*cloud_A);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits(-0.0,0.5);
  pass.filter (*cloud_A);

  std::cout<<"After pre-filtering there are "<<cloud_A->width * cloud_A->height << " data points in the lidar cloud. "<< std::endl;

  std::cerr << "BEGINNING ICP" << std::endl;
  // perform ICP on the lidar and cad clouds
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::PointCloud<pcl::PointXYZ> Final;

  //pcl::copyPointCloud(cloud_source,*cloud);

  icp.setInputTarget(cloud_A); // target (fixed) cloud
  icp.setInputCloud(cloud_B);  // source (moved during ICP) cloud

  icp.align(Final);

  Eigen::MatrixXf T_result;
  T_result=icp.getFinalTransformation();

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;
  std::cout << T_result << std::endl;

  std::cerr << "ICP COMPLETED" << std::endl;

  // instantiate a quaternion to copy Transformation to
  // this quaternion will be used to set markers rotation in RVIZ
  tf::Quaternion q_result;
  // use last collum of Transformation as center of marker
  result.setOrigin(tf::Vector3(T_result(0,3),T_result(1,3),T_result(2,3)));
  // instantiate a 3x3 rotation matrix from the transformation matrix
  tf::Matrix3x3 R_result(T_result(0,0),T_result(0,1),T_result(0,2),T_result(1,0),T_result(1,1),T_result(1,2),T_result(2,0),T_result(2,1),T_result(2,2));
  // copy tf::quaternion to q_result
  R_result.getRotation(q_result);
  // set set rotation of the frame for the marker
  result.setRotation(q_result);


  //pcl::copyPointCloud(cloud_source,*cloudB);

  std::cerr << "END OF REGISTER_CLOUD FUNCTION" << std::endl;
}


int main(int argc, char** argv)
{

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  // setup a tf for a 'searchbox' marker so we we can see it in RVIZ - maybe someday...
  // static tf::TransformBroadcaster br_searchbox;
  // tf::Transform tf_searchbox;

  // read the command line arguments to pick the data file and some other details
  std::string file_lidar = argv[2]; // source cloud
  std::string file_cad = argv[3];   // reference cloud
  double thresh = atof(argv[4]);

  // instantiate some clouds

  PointCloud::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZ>); // target cloud

  PointCloud::Ptr cloud_cad1 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud
  PointCloud::Ptr cloud_cad2 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud
  PointCloud::Ptr cloud_cad3 (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud

  PointCloud::Ptr cloud_part1 (new pcl::PointCloud<pcl::PointXYZ>);  // input to ransac
  PointCloud::Ptr cloud_part2 (new pcl::PointCloud<pcl::PointXYZ>); // input to ICP


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
      cloud_cad1->width * cloud_cad1->height << " Data points from "<< file_cad << std::endl;

  // setup a tf for a frame for the result so we we can see it in RVIZ
  static tf::TransformBroadcaster br_T1; // first pass
  static tf::TransformBroadcaster br_T1_inv; // first pass
  static tf::TransformBroadcaster br_T2; // first pass
  static tf::TransformBroadcaster br_T2_inv; // first pass
  static tf::TransformBroadcaster br_T3; // first pass
  //tf::Transform *tf_result; first pass
  tf::Transform *T1 (new tf::Transform);
  tf::Transform *T1_inv (new tf::Transform);
  tf::Transform *T2 (new tf::Transform);
  tf::Transform *T2_inv (new tf::Transform);
  tf::Transform *T3 (new tf::Transform);

  //static tf::TransformBroadcaster br_result2; //second pass
  //tf::Transform *tf_result; second pass
  //tf::Transform *tf_result2 (new tf::Transform);

  // RANSAC Segmentation to separate clouds
  segment_cloud(*cloud_lidar,*cloud_part1,*cloud_part2);
  // perform ICP Cloud Registration - results is a TF
  register_cloud(*cloud_cad1, *cloud_part1, *T1);

  *T1_inv=T1->inverse(); // invert the transform


  br_T1_inv.sendTransform(tf::StampedTransform(*T1_inv,ros::Time::now(),"T1_inv","map"));

  // now move the CAD part to the newly located frame
  pcl_ros::transformPointCloud(*cloud_cad1,*cloud_cad2,*T1_inv);

  // repeat registration
  register_cloud(*cloud_cad2, *cloud_part1, *T2);

  *T2_inv=T2->inverse(); // invert the transform

  br_T2_inv.sendTransform(tf::StampedTransform(*T2_inv,ros::Time::now(),"T2_inv","map"));

  pcl_ros::transformPointCloud(*cloud_cad2,*cloud_cad3,*T2_inv);
  //br_part1_inv.sendTransform(tf::StampedTransform(*T3,ros::Time::now(),"T1_inv","map"));

  // compute the final frame
  *T3=(*T1)*(*T2); // multiply the two transforms

  br_T3.sendTransform(tf::StampedTransform(*T3,ros::Time::now(),"T3","map"));

  ros::Publisher pub_lidar = node.advertise<PointCloud> ("/cloud_lidar", 1) ;
  ros::Publisher pub_cad1 = node.advertise<PointCloud> ("/cloud_cad1", 1) ;
  ros::Publisher pub_cad2 = node.advertise<PointCloud> ("/cloud_cad2", 1) ;
  ros::Publisher pub_cad3 = node.advertise<PointCloud> ("/cloud_cad3", 1) ;
  ros::Publisher pub_part1 = node.advertise<PointCloud> ("/cloud_part1", 1) ;
  ros::Publisher pub_part2 = node.advertise<PointCloud> ("/cloud_part2", 1) ;



  cloud_lidar->header.frame_id = "map";
  cloud_cad1->header.frame_id = "map";
  cloud_cad2->header.frame_id = "map";
  cloud_cad3->header.frame_id = "map";
  cloud_part1->header.frame_id = "map";
  cloud_part2->header.frame_id = "map";

  //publish forever
  while(ros::ok())
  {
      //br_T1.sendTransform(tf::StampedTransform(*T1,ros::Time::now(),"T1","map"));
      //br_T2.sendTransform(tf::StampedTransform(*T2,ros::Time::now(),"T2","map"));
      //br_T3.sendTransform(tf::StampedTransform(*T3,ros::Time::now(),"T3","map"));

      pub_lidar.publish(cloud_lidar);
      pub_cad1.publish(cloud_cad1);
      pub_cad2.publish(cloud_cad2);
      pub_cad2.publish(cloud_cad3);
      pub_part1.publish(cloud_part1);
      pub_part2.publish(cloud_part2);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

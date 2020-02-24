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

#include <tf/LinearMath/Matrix3x3.h>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Core>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


// make a function to organize the workflow, it is getting pretty strung out
void register_cloud(PointCloud &cloud_target, PointCloud &cloud_source, PointCloud &cloud_A, PointCloud &cloud_B, tf::Transform &result)
{

  // make a copy of the lidar cloud called 'cloud'
  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_target,*cloud);

  // XYZ Box Filter cloud before segementation
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(-0.5,0.5);
  pass.filter (*cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(-1.0,1.0);
  pass.filter (*cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits(-0.5,0.5);
  pass.filter (*cloud);

  std::cout<<"After pre-filtering there are "<<cloud->width * cloud->height << " data points in the lidar cloud. "<< std::endl;

  pcl::copyPointCloud(*cloud,cloud_A); // save input to RANSAC algorithm
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
  std::cerr << "RANSAC SEGEMENTATION COMPLETED" << std::endl;

  std::cerr << "BEGINNING ICP" << std::endl;
  // perform ICP on the lidar and cad clouds
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::PointCloud<pcl::PointXYZ> Final;

  pcl::copyPointCloud(cloud_source,*cloud);
  pcl::copyPointCloud(*cloud_cylinder,cloud_B); // save input to ICP algorithm

  icp.setInputCloud(cloud_cylinder);
  icp.setInputTarget(cloud);
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
  PointCloud::Ptr cloud_cad (new pcl::PointCloud<pcl::PointXYZ>);  // source cloud
  PointCloud::Ptr cloud_ransac (new pcl::PointCloud<pcl::PointXYZ>);  // input to ransac
  PointCloud::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZ>); // input to ICP


  // load the clouds from file (.pcd)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_lidar, *cloud_lidar) == -1)
  {
      std::cout<<"Couldn't read image file:"<<file_lidar;
      return (-1);
  }
  std::cout << "Loaded image file: "<< file_lidar <<std::endl<<
      cloud_lidar->width * cloud_lidar->height << " Data points from "<< file_lidar << std::endl;

  // load the cloud from CAD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_cad, *cloud_cad) == -1)
  {
      std::cout<<"Couldn't read image file:"<<file_cad;
      return (-1);
  }
  std::cout << "Loaded image file: "<< file_cad <<std::endl<<
      cloud_cad->width * cloud_cad->height << " Data points from "<< file_cad << std::endl;

  // setup a tf for a frame for the result so we we can see it in RVIZ
  static tf::TransformBroadcaster br_result;

  //tf::Transform *tf_result;
  tf::Transform *tf_result (new tf::Transform);

  // performs RANSAC Segmentation + ICP Cloud Registration - results is a TF
  register_cloud(*cloud_lidar, *cloud_cad, *cloud_ransac, *cloud_icp, *tf_result);

  ros::Publisher pub_lidar = node.advertise<PointCloud> ("/cloud_lidar", 1) ;
  ros::Publisher pub_cad = node.advertise<PointCloud> ("/cloud_cad", 1) ;
  ros::Publisher pub_ransac = node.advertise<PointCloud> ("/cloud_ransac", 1) ;
  ros::Publisher pub_icp = node.advertise<PointCloud> ("/cloud_icp", 1) ;

  // broadcast the transform to show the result*cloud_lidar,*cloud_cad
  br_result.sendTransform(tf::StampedTransform(*tf_result,ros::Time::now(),"result","map"));

  cloud_lidar->header.frame_id = "map";
  cloud_cad->header.frame_id = "map";
  cloud_ransac->header.frame_id = "map";
  cloud_icp->header.frame_id = "map";

  //publish forever
  while(ros::ok())
  {
      pub_lidar.publish(cloud_lidar);
      pub_cad.publish(cloud_cad);
      pub_ransac.publish(cloud_ransac);
      pub_icp.publish(cloud_icp);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

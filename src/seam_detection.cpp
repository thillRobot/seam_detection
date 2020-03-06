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
#include <pcl_ros/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
//#include <tf/TransformStamped.h>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void segment_cloud(PointCloud &cloud_input, PointCloud &cloud_output1, PointCloud &cloud_output2, pcl::ModelCoefficients &C_plane, pcl::ModelCoefficients &C_cylinder)
{
  // make a copy of the lidar cloud called 'cloud'
  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(cloud_input,*cloud);

  // XYZ Box Filter cloud before segementation
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(0.0,0.5);
  pass.filter (*cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(0.0,0.5);
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
void register_cloud(PointCloud &cloud_target, PointCloud &cloud_source, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA, geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA)
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

  Eigen::MatrixXf T_result;
  Eigen::MatrixXf T_inverse;

  icp.setMaximumIterations(20);// the default is 10
  icp.setInputTarget(cloud_A); // target (fixed) cloud
  icp.setInputCloud(cloud_B);  // source (moved during ICP) cloud
  icp.align(Final);

  T_result=icp.getFinalTransformation(); // get the resutls of ICP
  T_inverse=T_result.inverse();

  //Eigen::MatrixXf *T_eig (new Eigen::MatrixXf);
  T_result=icp.getFinalTransformation(); // get the resutls of ICP


  std::cerr << "ICP COMPLETED" << std::endl;
  std::cout << "max iterations:" << icp.getMaximumIterations() << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << T_result << std::endl;

  //tf::Transform *T (new tf::Transform);
  //tf::transformEigenToTF(T_result,*T);

  // this part seems very over bloated !!!
  // use last collum of Transformation as center of marker
  tf::Quaternion q_result;
  tf2::Quaternion *q_result_tf2 (new tf2::Quaternion);

  tf::Quaternion q_inverse;
  tf2::Quaternion *q_inverse_tf2 (new tf2::Quaternion);
  // instantiate a 3x3 rotation matrix from the transformation matrix
  tf::Matrix3x3 R_result(T_result(0,0),T_result(0,1),T_result(0,2),T_result(1,0),T_result(1,1),T_result(1,2),T_result(2,0),T_result(2,1),T_result(2,2));
  tf2::Matrix3x3 R_result_tf2(T_result(0,0),T_result(0,1),T_result(0,2),T_result(1,0),T_result(1,1),T_result(1,2),T_result(2,0),T_result(2,1),T_result(2,2));

  tf::Matrix3x3 R_inverse(T_inverse(0,0),T_inverse(0,1),T_inverse(0,2),T_inverse(1,0),T_inverse(1,1),T_inverse(1,2),T_inverse(2,0),T_inverse(2,1),T_inverse(2,2));
  tf2::Matrix3x3 R_inverse_tf2(T_inverse(0,0),T_inverse(0,1),T_inverse(0,2),T_inverse(1,0),T_inverse(1,1),T_inverse(1,2),T_inverse(2,0),T_inverse(2,1),T_inverse(2,2));

  // copy tf::quaternion from R_result to q_result
  R_result.getRotation(q_result);
  R_result_tf2.getRotation(*q_result_tf2);

  // copy tf::quaternion from R_result to q_result
  R_inverse.getRotation(q_inverse);
  R_inverse_tf2.getRotation(*q_inverse_tf2);

  // set set rotation and origin of a quaternion for the tf transform object
  T_AB.setRotation(q_result);
  T_AB.setOrigin(tf::Vector3(T_result(0,3),T_result(1,3),T_result(2,3)));

  //T_AB.setRotation(R_result.getRotation());
  //T_AB.setOrigin(tf::Vector3(T_result(0,3),T_result(1,3),T_result(2,3)));


  // new 'TF2' style tf transform object
  q_result_tf2->normalize(); // normalize the Quaternion
  //tf2_out.setRotation(*q_result_tf2);
  //tf2_out.setOrigin(tf2::Vector3(T_result(0,3),T_result(1,3),T_result(2,3)));

  // set set rotation and origin of a quaternion for the tf transform object
  // the message also needs header info, a name and parent
  /*
  //msg_out.header.stamp = ros::Time::now();
  //msg_AB.header.frame_id = "base_link";
  msg_AB.transform.translation.x = T_result(0,3);
  msg_AB.transform.translation.y = T_result(1,3);
  msg_AB.transform.translation.z = T_result(2,3);
  //  TF2 is  used for the tf message

  msg_AB.transform.rotation.x = q_result_tf2->x();
  msg_AB.transform.rotation.y = q_result_tf2->y();
  msg_AB.transform.rotation.z = q_result_tf2->z();
  msg_AB.transform.rotation.w = q_result_tf2->w();
  */

  // set set rotation and origin tfof a quaternion for the tf transform object
  T_BA.setRotation(q_inverse);
  T_BA.setOrigin(tf::Vector3(T_inverse(0,3),T_inverse(1,3),T_inverse(2,3)));
  // new 'TF2' style tf transform object
  q_inverse_tf2->normalize(); // normalize the Quaternion

  /*
  //msg_BA.header.frame_id = "base_link";
  msg_BA.transform.translation.x = T_inverse(0,3);
  msg_BA.transform.translation.y = T_inverse(1,3);
  msg_BA.transform.translation.z = T_inverse(2,3);
  //  TF2 is  used for the tf message

  msg_BA.transform.rotation.x = q_inverse_tf2->x();
  msg_BA.transform.rotation.y = q_inverse_tf2->y();
  msg_BA.transform.rotation.z = q_inverse_tf2->z();
  msg_BA.transform.rotation.w = q_inverse_tf2->w();
  */

  tf::transformStampedTFToMsg(T_AB,msg_AB);
  tf::transformStampedTFToMsg(T_BA,msg_BA);

  /* // this part causes normaize errors
  msg_out.transform.rotation.x = tf2_out.getRotation().normalized().getAxis().getX();
  msg_out.transform.rotation.y = tf2_out.getRotation().normalized().getAxis().getY();
  msg_out.transform.rotation.z = tf2_out.getRotation().normalized().getAxis().getZ();
  msg_out.transform.rotation.w = tf2_out.getRotation().normalized().getW();
  */

  //  TF is coud be used for the tf message
  /*
  msg_out.transform.rotation.x = q_result.x();
  msg_out.transform.rotation.y = q_result.y();
  msg_out.transform.rotation.z = q_result.z();
  msg_out.transform.rotation.w = q_result.w();
  */
  std::cerr << "END OF REGISTER_CLOUD FUNCTION" << std::endl;
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

  //geometry_msgs::TransformStamped msg;

  //geometry_msgs::TransformStamped *msg (new geometry_msgs::TransformStamped);

  tf::transformStampedTFToMsg(T_AC,msg_AC);
  tf::transformStampedTFToMsg(T_CA,msg_CA);

}

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
*/

/*
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

  // setup a tf for a 'searchbox' marker so we we can see it in RVIZ - maybe someday...
  // static tf::TransformBroadcaster br_searchbox;
  // tf::Transform tf_searchbox;

  // read the command line arguments to pick the data file and some other details
  std::string file_lidar = argv[2]; // source cloud
  std::string file_cad = argv[3];   // reference cloud
  double thresh = atof(argv[4]);

  // instantiate some clouds
  PointCloud::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZ>); // target cloud  // inputs to RANSAC
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
      cloud_cad1->width * cloud_cad1->height << " Data points from "<< file_cad << std::endl;

  // for now each tf has three objects associated with it
  // 1) 'name' (tf::transform)      // needed for transforms with pcl_ros
  // 2) 'name_tf2' (tf2::transform) // not used
  // 3) 'name_msg' (geometry_msgs)  // needed for bradcasting frames

  tf::StampedTransform *T_01 (new tf::StampedTransform);    // these are from the old 'TF'
  tf::StampedTransform *T_10 (new tf::StampedTransform);    // they are stil used for pcl_ros::transformPointCloud

  tf::StampedTransform *T_12 (new tf::StampedTransform);
  tf::StampedTransform *T_21 (new tf::StampedTransform);

  tf::StampedTransform *T_02 (new tf::StampedTransform);
  tf::StampedTransform *T_20 (new tf::StampedTransform);

  //tf::StampedTransform *T_01_s (new tf::StampedTransform);
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

  geometry_msgs::TransformStamped *T_12_msg (new geometry_msgs::TransformStamped);
  T_12_msg->header.frame_id = "base_link"; T_12_msg->child_frame_id = "T_12";

  geometry_msgs::TransformStamped *T_21_msg (new geometry_msgs::TransformStamped);
  T_21_msg->header.frame_id = "base_link"; T_21_msg->child_frame_id = "T_21";

  geometry_msgs::TransformStamped *T_02_msg (new geometry_msgs::TransformStamped);
  T_02_msg->header.frame_id = "base_link"; T_02_msg->child_frame_id = "T_02";

  geometry_msgs::TransformStamped *T_20_msg (new geometry_msgs::TransformStamped);


  pcl::ModelCoefficients::Ptr coeffs_plane (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coeffs_cylinder (new pcl::ModelCoefficients);

  // RANSAC Segmentation to separate clouds
  segment_cloud(*cloud_lidar,*cloud_part1,*cloud_part2,*coeffs_plane,*coeffs_cylinder);

  // perform ICP Cloud Registration - results is a TF
  register_cloud(*cloud_cad1, *cloud_part1,*T_10, *T_01, *T_10_msg, *T_01_msg);

  std::cout<<"Computing Matrix Inverse"<<std::endl;

  // now move the CAD part to the newly located frame
  pcl_ros::transformPointCloud(*cloud_cad1,*cloud_cad2,*T_01); // this works with 'pcl::PointCloud<pcl::PointXYZ>' and 'tf::Transform'
  std::cerr << "Cloud transformed." << std::endl;
  //tf2::doTransform(*cloud_cad1,*cloud_cad2,*T_01_msg); // I have not made this work yet...

  // repeat registration on moved cad model
  register_cloud(*cloud_cad2, *cloud_part1, *T_21, *T_12, *T_21_msg, *T_12_msg);

  // now move the CAD part again to the newly located frame
  pcl_ros::transformPointCloud(*cloud_cad2,*cloud_cad3,*T_12);
  std::cerr << "Cloud transformed again." << std::endl;


  //*T_02=(*T_01)*(*T_12); // multiply the two transforms to get final tf
  //*T_20=T_02->inverse(); // get the inverse of the tf

  combine_transformation(*T_01,*T_12,*T_20,*T_02,*T_20_msg,*T_02_msg);
  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";

  T_12_msg->header.frame_id = "base_link"; T_12_msg->child_frame_id = "T_12";
  T_21_msg->header.frame_id = "base_link"; T_21_msg->child_frame_id = "T_21";

  T_02_msg->header.frame_id = "base_link"; T_02_msg->child_frame_id = "T_02";
  T_20_msg->header.frame_id = "base_link"; T_20_msg->child_frame_id = "T_20";

  std::cerr << "Final transformation computed and converted to message." << std::endl;

  // publish 'markers' to to show the plane and cylinder found with RANSAC
  //pubs for the plane marker
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
  marker_plane.pose.orientation.x = 0;  // set rotation
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

  //pubs for the cylinder marker
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
  ros::Publisher pub_cad1 = node.advertise<PointCloud> ("/cloud_cad1", 1) ;
  ros::Publisher pub_cad2 = node.advertise<PointCloud> ("/cloud_cad2", 1) ;
  ros::Publisher pub_cad3 = node.advertise<PointCloud> ("/cloud_cad3", 1) ;
  ros::Publisher pub_part1 = node.advertise<PointCloud> ("/cloud_part1", 1) ;
  ros::Publisher pub_part2 = node.advertise<PointCloud> ("/cloud_part2", 1) ;

  cloud_lidar->header.frame_id = "base_link";
  cloud_cad1->header.frame_id = "base_link";
  cloud_cad2->header.frame_id = "base_link";
  cloud_cad3->header.frame_id = "base_link";
  cloud_part1->header.frame_id = "base_link";
  cloud_part2->header.frame_id = "base_link";

  //publish forever
  while(ros::ok())
  {
      // this is the old 'TF' way, it doesnt work so I dont know why I am leaving it here
      //T_br.sendTransform(tf::StampedTransform(*T1,ros::Time::now(),"T1","map"));

      // this is the new 'TF2' way of broadcasting tfs
      T_01_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_msg);
      T_10_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_msg);

      T_12_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_12_msg);
      T_21_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_21_msg);

      T_02_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_02_msg);
      T_20_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_20_msg);

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

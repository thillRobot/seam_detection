/*
RANSAC/Segementation based multiple plane detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University

Taken from PCL sample code - 02/14/2018
Updated - 02/17/2018

Revisited 02/22/2020

Added 'cylinder segmentation'

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

int main(int argc, char** argv)
{
  //PointCloud::Ptr cloud_in (new PointCloud);    //save a copy of the original
  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy
  //PointCloud::Ptr cloud_out1 (new PointCloud);  //these are the clouds for the planes
  //PointCloud::Ptr cloud_out2 (new PointCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  ros::init(argc,argv,"seam_detection_RANSAC_ICP");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  ros::Publisher pub_lidar = node.advertise<PointCloud> ("/lidar_cloud", 1) ;
  ros::Publisher pub_cad = node.advertise<PointCloud> ("/cad_cloud", 1) ;
  ros::Publisher pub_cylinder = node.advertise<PointCloud> ("/cloud_cylinder", 1) ;
  ros::Publisher pub_plane = node.advertise<PointCloud> ("/cloud_plane", 1) ;
  //ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_out1", 1) ;
  //ros::Publisher pub3 = node.advertise<PointCloud> ("/cloud_out2", 1) ;

  // setup a tf for a fram for the result so we we can see it in RVIZ
  static tf::TransformBroadcaster br_result;
  tf::Transform tf_result;

  // read the command line arguments to pick the data file and some other details
  std::string lidar_file = argv[2]; // source cloud
  std::string cad_file = argv[3];   // reference cloud
  double thresh = atof(argv[4]);

  // paths come from command line args
  std::string lidar_cloud_path = lidar_file;
  std::string cad_cloud_path = cad_file;

  // load the cloud from lidar file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_cloud_path, *lidar_cloud) == -1)
  {
      std::cout<<"Couldn't read image file:"<<lidar_cloud_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< lidar_cloud_path <<std::endl<<
      lidar_cloud->width * lidar_cloud->height << " Data points from "<< lidar_file << std::endl;

  // load the cloud from CAD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (cad_cloud_path, *cad_cloud) == -1)
  {
      std::cout<<"Couldn't read image file:"<<cad_cloud_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< cad_cloud_path <<std::endl<<
      cad_cloud->width * cad_cloud->height << " Data points from "<< cad_file << std::endl;


    pcl::copyPointCloud(*lidar_cloud,*cloud);
    // Filter cloud before segementation
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

    // All the objects needed
    pcl::PCDReader reader;
    //pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

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
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
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
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
      std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
      std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
      writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }

    // perform ICP on the lidar and cad clouds
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ> Final;

    icp.setInputCloud(cloud_cylinder);
    icp.setInputTarget(cad_cloud);
    icp.align(Final);

    Eigen::MatrixXf T_result;
    T_result=icp.getFinalTransformation();

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << T_result << std::endl;

    // instantiate a quaternion to copy Transformation to
    // this quaternion will be used to set markers rotation in RVIZ
    tf::Quaternion q_result;
    // use last collum of Transformation as center of marker
    tf_result.setOrigin(tf::Vector3(T_result(0,3),T_result(1,3),T_result(2,3)));
    // instantiate a 3x3 rotation matrix from the transformation matrix
    tf::Matrix3x3 R_result(T_result(0,0),T_result(0,1),T_result(0,2),T_result(1,0),T_result(1,1),T_result(1,2),T_result(2,0),T_result(2,1),T_result(2,2));
    // copy tf::quaternion to q_result
    R_result.getRotation(q_result);
    // set set rotation of the frame for the marker
    tf_result.setRotation(q_result);
    // broadcast the transform to show the result
    br_result.sendTransform(tf::StampedTransform(tf_result,ros::Time::now(),"result","map"));

    //pcl::copyPointCloud(*cloud_cylinder,*cloud_out1);
    //pcl::copyPointCloud(*cloud_plane,*cloud_out2);

    lidar_cloud->header.frame_id = "map";
    cad_cloud->header.frame_id = "map";
    cloud_cylinder->header.frame_id = "map";
    cloud_plane->header.frame_id = "map";

    //publish forever
    while(ros::ok())
    {
        pub_lidar.publish(lidar_cloud);
        pub_cad.publish(cad_cloud);
        pub_cylinder.publish(cloud_cylinder);
        pub_plane.publish(cloud_plane);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

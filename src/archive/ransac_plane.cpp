/*
    RANSAC based plane detection using PCL

    Tristan Hill - Weld Seam Detection - Tennessee Technological University

    Taken from PCL sample code - 02/14/2018

    Robotics Research Group - Mechanical Engineering
*/

#include <iostream>
#include <string>

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

#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char** argv)
{
  
  std::cout<<"*************************************************************"<<endl;
  std::cout<<"******************** RANSAC Plane v1.4 ********************"<<endl;
  std::cout<<"*************************************************************"<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  ros::init(argc,argv,"RANSAC Plane Detection Node");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<PointCloud> ("/cloud_ransac", 1) ;
  ros::Rate loop_rate(10);

  PointCloud::Ptr cloud_pub (new PointCloud);
  PointCloud::Ptr cloud (new PointCloud);

  // read the command line arguments to pick the data file
  std::string in_file = argv[2];

  // This path must be changed when I switch workstations - TWH
  //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/scan2cloud/images/")+in_file;
  std::string test_path = in_file;

  // load the cloud from file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (test_path, *cloud) == -1)
  {
    std::cout<<"Couldn't read image file:"<<test_path;
    return (-1);
  }
  std::cout << "Loaded image file: "<< test_path <<std::endl<<
            cloud->width * cloud->height << " Data points from "<< in_file << std::endl;

  std::vector<int> inliers;

  // instantiate RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);

  ransac.setDistanceThreshold (.01);
  ransac.computeModel();
  ransac.getInliers(inliers);

  std::vector< int > model;
  ransac.getModel(model);

  std::cout<<"The model VectorXf: "<<std::endl ;
  for (std::vector<int>::const_iterator i = model.begin(); i != model.end(); ++i)
    std::cout << *i << ' ';
  std::cout<<std::endl;
  //std::cout<<"The model is:"<< std::string(model) << std::endl;

  //pcl::SacModel model_type;
  //model_type=model_p.getModelType();
  //std::cout<<"The model type is:"<< model_type << std::endl;

  Eigen::VectorXf coefs;
  ransac.getModelCoefficients(coefs);
  std::cout<<"The coefficients of the model:"<<std::endl<< coefs << std::endl;

  // copy all inliers of the model computed to the PointCloud for to publish
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_pub);

  cloud_pub->header.frame_id = "map";
  //cloud_pub->height = cloud_pub->width = 1;
  //pcl_conversions::fromPCL(cloud_ransac, cloud_pub);

  while(ros::ok())
  {
    pub.publish(cloud_pub);
    ros::spinOnce();
    loop_rate.sleep();
  }

  /*
  // Uncomment this block if you want to use the '3d Viewer' to see the clouds
  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.

  // copy all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0)
    viewer = simpleVis(cloud_final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  */

  return 0;
 }

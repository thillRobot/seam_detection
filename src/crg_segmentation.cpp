// this program loads and processes RGB PointClouds and processes them with Color Based Region Growing Segmentation 
// using pcl::RegionGrowingRGB, https://pointclouds.org/documentation/classpcl_1_1_region_growing_r_g_b.html
// adapted from example code in tutorial link below 
// https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_rgb_segmentation.html

#include <iostream>
#include <thread>
#include <vector>
#include <Eigen/Core>

#include "ros/package.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
//#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std::chrono_literals;

typedef pcl::PointCloud< pcl::PointXYZRGB > PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;
typedef pcl::PointIndices::Ptr PointIndicesPtr;


// function to copy PointCloud with XYZRGB points
// it seems like this would be in pcl, but i get a type error with PointXYZRGB
void copy_cloud(PointCloud &input, PointCloud &output){

  std::cout<<"the point cloud input has "<< input.size()<< " points"<<std::endl;

  for (int i=0; i<input.size(); i++) { // add points to cluster cloud
    output.push_back(input[i]);  
  } 

  std::cout<<"the point cloud output has "<< output.size()<< " points"<<std::endl;

}


int main(int argc, char** argv)
{

  ros::init(argc,argv,"crg_segmentation");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     crg_segmentation v1.x                             "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     crg_segmentation: loading configuration file       "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // this is not true, the params are loaded in the launch file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters 
  bool save_output, translate_output, automatic_bounds, transform_input;
  node.getParam("save_output", save_output);
  node.getParam("translate_output", translate_output);
  node.getParam("automatic_bounds", automatic_bounds);
  node.getParam("transform_input", transform_input);

  // parameters that contain strings  
  std::string input_path, output_path, input_file, output_file; 
  node.getParam("crg_segmentation/input_file", input_file);
  input_path=packagepath+'/'+input_file;
  node.getParam("crg_segmentation/output_file", output_file);
  output_path=packagepath+'/'+output_file;

  // parameters that contain doubles 
  double voxel_leaf_size, cluster_tolerance;
  node.getParam("crg_segmentation/voxel_leaf_size", voxel_leaf_size);
  node.getParam("crg_segmentation/cluster_tolerance", cluster_tolerance);

  // parameters that contain ints
  int cluster_min_size, cluster_max_size;
  node.getParam("crg_segmentation/cluster_min_size", cluster_min_size);
  node.getParam("crg_segmentation/cluster_max_size", cluster_max_size);

  // parameters that contain vectors of doubles
  std::vector<double> filter_box_vec;
  double filter_box[6], box[6];

  std::vector<double> pre_rotation_vec, pre_translation_vec;
  Eigen::Vector3f pre_rotation, pre_translation;

  node.getParam("crg_segmentation/pre_rotation",  pre_rotation_vec);
  node.getParam("crg_segmentation/pre_translation",  pre_translation_vec);
  
  for(unsigned i=0; i < pre_rotation_vec.size(); i++){
    pre_rotation[i]=pre_rotation_vec[i]; // copy from std vector to eigen vector3f 
    pre_translation[i]=pre_translation_vec[i]; 
  }

  node.getParam("crg_segmentation/filter_box",  filter_box_vec);
  for(unsigned i=0; i < filter_box_vec.size(); i++)
    filter_box[i]=filter_box_vec[i]; // copy from vector to array 

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     crg_segmentation: preparing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
 
  // instantiate some cloud pointers
  PointCloud::Ptr cloud_input (new PointCloud); 
  PointCloud::Ptr cloud (new PointCloud);      // working copy for this routine
  PointCloud::Ptr cloud_copy (new PointCloud);

  std::cout << "Loading cloud input file:" << input_path << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_path, *cloud_input) == -1)
  {
      std::cout<<"Couldn't read cloud input file:"<<input_path;
      return (-1);
  }
  std::cout << "Loaded cloud input file: "<< input_path <<std::endl<<
    cloud_input->width * cloud_input->height << " Data points from "<< input_path << std::endl;
 
  // make a working copy of the cloud input called 'cloud'
  copy_cloud(*cloud_input, *cloud);

  // transform the pointcloud
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
 
  // Define a translation 
  transform.translation() << pre_translation[0], pre_translation[1], pre_translation[2];
  // define three axis rotations (RPY)
  transform.rotate (Eigen::AngleAxisf (pre_rotation[0], Eigen::Vector3f::UnitX()));
  transform.rotate (Eigen::AngleAxisf (pre_rotation[1], Eigen::Vector3f::UnitY()));
  transform.rotate (Eigen::AngleAxisf (pre_rotation[2], Eigen::Vector3f::UnitZ()));

  // Execute the transformation, update the working copy of the cloud
  pcl::transformPointCloud (*cloud, *cloud, transform); 

  std::cout<<"After pre-translation and pre-rotation there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;


  //Apply Bounding Box Filter 
  std::cout<<"filter_box: [ " <<filter_box[0]<<", "<<filter_box[1]<<", "
                              <<filter_box[2]<<", "<<filter_box[3]<<", "
                              <<filter_box[4]<<", "<<filter_box[5]<<" ]" << std::endl;

  pcl::PassThrough<pcl::PointXYZRGB> pass; 
  pass.setInputCloud(cloud);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits(filter_box[0],filter_box[1]);
  pass.filter (*cloud);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits(filter_box[2],filter_box[3]);
  pass.filter (*cloud);

  pass.setFilterFieldName ("z");
  pass.setFilterLimits(filter_box[4],filter_box[5]);
  pass.filter (*cloud);
    
  std::cout<<"After bounding box filter there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;

  // Apply Voxel Filter 
  if (voxel_leaf_size>0)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size); // use "001f","001f","0001f" or "none" to set voxel leaf size
    vox.filter (*cloud);
    std::cout<<"After voxel filtering there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
  }else
  {
    std::cout<<"No voxel filtering"<< std::endl;
  }
  
  // perform color based region growing segmentation  
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud (*cloud, *indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;

  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (10);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");

  viewer.showCloud (colored_cloud);

  while (!viewer.wasStopped ())
  {
    std::this_thread::sleep_for(100us); // 100us from std::chrono_literals
  }

  return (0);

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                        crg_segmentation: complete                  "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;
}
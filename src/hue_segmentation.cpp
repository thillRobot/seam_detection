// this program loads and processes RGB PointClouds and processes them with seeded hue segmentation

// using pcl::SeededHueSegmentation, 
// http://docs.ros.org/en/hydro/api/pcl/html/classpcl_1_1SeededHueSegmentation.html

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/seeded_hue_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include "ros/package.h"
#include <ros/ros.h>


typedef pcl::PointCloud< pcl::PointXYZRGB > PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;

typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;
typedef pcl::PointIndices::Ptr PointIndicesPtr;

int main(int argc, char** argv)
{

  ros::init(argc,argv,"hue_segmentation");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     hue_segmentation v1.x                             "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     hue_segmentation: loading configuration file       "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // this is not true, the params are loaded in the launch file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_cloud: preparing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  
  // parameters that contain strings  
  std::string input_path, input_file; 
  node.getParam("hue_segmentation/input_file", input_file);
  input_path=packagepath+'/'+input_file;

  // instantiate some cloud pointers
  PointCloud::Ptr cloud_input (new PointCloud); 

  std::cout << "Loading cloud input file:" << input_path << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_path, *cloud_input) == -1)
  {
      std::cout<<"Couldn't read cloud input file:"<<input_path;
      return (-1);
  }
  std::cout << "Loaded cloud input file: "<< input_path <<std::endl<<
    cloud_input->width * cloud_input->height << " Data points from "<< input_path << std::endl;
 

  //PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  //pcl::copyPointCloud(cloud_input, *cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_input);

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::PointIndices::Ptr indices_in (new pcl::PointIndices ());
  
  pcl::PointIndices::Ptr indices_out (new pcl::PointIndices ());
  
  //pcl::SeededHueSegmentation<pcl::PointXYZRGB> shs;
  pcl::SeededHueSegmentation shs;
  float tolerance=0.02;
  shs.setClusterTolerance (tolerance); // cluster parameters set in config file
  shs.setSearchMethod (tree);
  shs.setInputCloud (cloud_input);
  //shs.extract (cluster_indices);
  shs.segment(*indices_in, *indices_out);

  // instatiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
  //PointCloudVec clusters;
  
  /*
  int j = 0;
  for (const auto& cluster : indices_out) 
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : cluster.indices) { // add points to cluster cloud
      cloud_cluster->push_back((*cloud_input)[idx]);
    } 
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //clusters.push_back(cloud_cluster); // add clusters to vector of clusters

    std::cout << "PointCloud representing cluster"<<j<<" has "<< cloud_cluster->size() << " data points " << std::endl;
    j++; // increment the cluster counter
  }
  */
  
  //for (int i = 0; i < clusters.size(); i++){
  //    std::cout << "Point Cloud " << i << " has " << clusters[i]->size() << " Points " << std::endl;
  //}

  //std::cout<< "clusters size: "<< clusters.size() <<std::endl;
  



  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                        hue_segmentation: complete                  "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;
}
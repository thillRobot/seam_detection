/*
Seam Detection, FilterCloud
This source contains the cpp class FilterCloud for this project
Created on 02/06/2024, this contains some useful tools for working with pointclouds

Tristan Hill - Weld Seam Detection - Tennessee Technological University
Robotics Research Group - Mechanical Engineering

see README.md or https://github.com/thillRobot/seam_detection for documentation
*/

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/pcl_config.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/surface/mls.h>

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

using namespace std::chrono_literals;

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

// PCL PointClouds with XYZ RGB Normal Points
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

// Vector of PointClouds
// aligned_allocator - STL compatible allocator to use with types requiring a non-standard alignment 
typedef std::vector < PointCloudPtr, Eigen::aligned_allocator < PointCloudPtr > > PointCloudVec;
typedef std::vector < PointCloudNormalPtr, Eigen::aligned_allocator < PointCloudNormalPtr > > PointCloudNormalVec;

class FilterCloud {

  public:

    // functions 
    
    // default constructor
    FilterCloud(): rate(5), pub_idx(0) { // ROS::Rate rate(5) is in intializer list
      
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"|---------- FilterCloud v1.9 ----------|"<<std::endl;
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

      // allocate memory for pointclouds member attributes
      input = new PointCloud;
      downsampled = new PointCloud;
      transformed = new PointCloud;
      bounded = new PointCloud; 
      smoothed = new PointCloud;      

      // find the path to the this package (seam_detection)
      package_path = ros::package::getPath("seam_detection");
  
    }
  

    // function to load the config file(yaml) to pick the data files and set parameters 
    int loadConfig(void){

      std::cout<<"|---------- SeamDetection::LoadConfig - loading configuration file ---------|"<<std::endl;

      // get boolen parameters 
      node.getParam("save_output", save_output);
      node.getParam("translate_output", translate_output);
      node.getParam("automatic_bounds", automatic_bounds);
      node.getParam("use_clustering", use_clustering);
      node.getParam("new_scan", new_scan);
      node.getParam("transform_input", transform_input);

      // get parameters that contain strings  
      node.getParam("input_file", input_file);    
      node.getParam("output_file", output_file);

      // generate absolute file paths to inputs (does this belong here?)
      input_path=package_path+'/'+input_file;
      output_path=package_path+'/'+output_file;
      
      // get parameters that contain doubles 
      node.getParam("voxel_size", voxel_size);

      // parameters that contain vectors of doubles
      std::vector<double> bounding_box_vec;
      node.getParam("bounding_box",  bounding_box_vec);
      for(unsigned i=0; i < bounding_box_vec.size(); i++){
        bounding_box[i]=bounding_box_vec[i]; // copy from vector to array 
      }

      // rotation and translation parameters from camera to fixed frame  
      std::vector<double> pre_rotation_vec, pre_translation_vec;
      node.getParam("pre_rotation",  pre_rotation_vec);
      node.getParam("pre_translation",  pre_translation_vec);
      for(unsigned i=0; i < pre_rotation_vec.size(); i++){
        pre_rotation[i]=pre_rotation_vec[i]; // copy from std vector to eigen vector3f 
        pre_translation[i]=pre_translation_vec[i]; 
      }
         
      return 0;
    }
    

    // templated function to load pcl::PointCloud<point_t> from PCD file as defined in config
    template <typename point_t>
    int loadCloud(std::string file, pcl::PointCloud<point_t> &input){

      std::cout<<"|---------- SeamDetection::LoadCloud - loading PCD file ----------|"<<std::endl;

      std::string path;
      path=package_path+"/"+file;

      std::cout << "Loading input pointcloud file: " << path << std::endl;
      if (pcl::io::loadPCDFile<point_t> (path, input) == -1)
      {
        std::cout<<"Failed to load input pointcloud file: "<< path <<std::endl;
        return (-1);
      }
      std::cout << "Loaded "<<input.width * input.height << " data points from input pointcloud file: "<< path <<std::endl;
      return 0;  
    } 
    
    // templated function to save pcl::PointCloud<point_t> to PCD file as defined in config
    template <typename point_t>
    int saveCloud(std::string file, pcl::PointCloud<point_t> &output){

      std::cout<<"|---------- SeamDetection::SaveCloud - saving PCD file ----------|"<<std::endl;

      std::string path;
      path=package_path+"/"+file;
      
      // save filtered cloud 
      
      //pcl::io::savePCDFileASCII (output_path, *cloud_filtered); // this might be the wrong file saved
      //std::cout<<"Filtered cloud written to:"<< output_path <<std::endl;
    
      std::cout << "Saving output pointcloud file: " << path << std::endl;
      if (  pcl::io::savePCDFileASCII(path, output) == -1)
      {
        std::cout<<"Failed to save ouput pointcloud file: "<< path <<std::endl;
        return (-1);
      }
      std::cout << "Saved "<<output.width * output.height << " data points to output pointcloud file: "<< path <<std::endl;
      return 0;  
    } 


    
    // templated function to publish a single pcl::PointCloud<point_t> as a ROS topic 
    template <typename point_t>
    void publishCloud(point_t &cloud, std::string topic){
      std::cout<<"|---------- SeamDetection::publishCloud - publishing single cloud ----------|"<<std::endl;

      // advertise a new topic and publish a msg each time this function is called
      pub_clouds.push_back(node.advertise<pcl::PointCloud<point_t>>(topic, 0, true));
      
      cloud.header.frame_id = "base_link";

      pub_clouds[pub_clouds.size()-1].publish(cloud);

      ros::spinOnce();

    }


    // function to publish a vector of PointClouds representing clusters as a ROS topic
    void publishClusters(PointCloudVec &clusters, std::string prefix){
      std::cout<<"|---------- SeamDetection::publishClusters - publishing clusters ----------|"<<std::endl;
        
      for (int i=0; i<clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster in clusters
        std::stringstream name;
        name << prefix << i;
        pub_clusters.push_back(node.advertise<PointCloud>(name.str(), 0, true));
        clusters[i]->header.frame_id = "base_link";
        pub_clusters[pub_idx].publish(clusters[i]);
        pub_idx++;
      }
      
      ros::spinOnce();
    }


    // function to publish a vector of PointClouds with normals representing clusters as a ROS topic
    void publishClusters(PointCloudNormalVec &clusters, std::string prefix){
      std::cout<<"|---------- SeamDetection::publishClusters - publishing clusters ----------|"<<std::endl;
        
      for (int i=0; i<clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster in clusters
        std::stringstream name;
        name << prefix << i;
        pub_clusters.push_back(node.advertise<PointCloudNormal>(name.str(), 0, true));
        clusters[i]->header.frame_id = "base_link";
        pub_clusters[pub_idx].publish(clusters[i]);
        pub_idx++;
      }
      
      ros::spinOnce();
    }


    
    // templated function to publish a vector of PointClouds with normals representing clusters as a ROS topic
    template <typename point_t>
    void publishClustersT(const std::vector<typename pcl::PointCloud<point_t>::Ptr, Eigen::aligned_allocator<typename pcl::PointCloud<point_t>::Ptr> > &clusters, std::string prefix){
      std::cout<<"|---------- SeamDetection::publishClusters - publishing clusters ----------|"<<std::endl;
        
      for (int i=0; i<clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster in clusters
        std::stringstream name;
        name << prefix << i;
        pub_clusters.push_back(node.advertise<point_t>>(name.str(), 0, true)); // this type needs handling too
        clusters[i]->header.frame_id = "base_link";
        pub_clusters[pub_idx].publish(clusters[i]);
        pub_idx++;
      }
      
      ros::spinOnce();
    }
    /*
    // templated function to publish a vector of PointClouds with normals representing clusters as a ROS topic
    template <typename T, typename A>
    void publishClustersT(std::vector< typename pcl::PointCloud<T>::Ptr,A >& clusters, std::string prefix){
      std::cout<<"|---------- SeamDetection::publishClusters - publishing clusters ----------|"<<std::endl;
        
      for (int i=0; i<clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster in clusters
        std::stringstream name;
        name << prefix << i;
        pub_clusters.push_back(node.advertise<T>>(name.str(), 0, true)); // this type needs handling too
        clusters[i]->header.frame_id = "base_link";
        pub_clusters[pub_idx].publish(clusters[i]);
        pub_idx++;
      }
      
      ros::spinOnce();
    }*/
 
    // function to copy PointCloud with XYZRGB points - not needed, use pcl::copyPointCloud()
    void copyCloud(PointCloud &input, PointCloud &output){

      std::cout<<"the point cloud input has "<< input.size()<< " points"<<std::endl;
      for (int i=0; i<input.size(); i++) { // add points to cluster cloud
        output.push_back(input[i]);  
      } 
      std::cout<<"the point cloud output has "<< output.size()<< " points"<<std::endl;
    
    }


    // function to apply voxel downsampling to pointcloud 
    void downsampleCloud(PointCloud &input, PointCloud &output, double leaf_size){

      PointCloud::Ptr cloud (new PointCloud); 
      pcl::copyPointCloud(input, *cloud);        // this copy ensures that the input data is left unchanged
 
      // Apply Voxel Filter 
      std::cout<<"Before voxel filtering there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
      if (leaf_size>0)
      {
        pcl::VoxelGrid<PointT> vox;
        vox.setInputCloud (cloud); // operate directly on the output PointCloud pointer, removes need for copy below
        vox.setLeafSize (leaf_size, leaf_size, leaf_size); // use "001f","001f","0001f" or "none" to set voxel leaf size
        vox.filter (*cloud);
        std::cout<<"After voxel filtering there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
      }else
      {
        std::cout<<"leaf_size>0 false, no voxel filtering"<< std::endl;
      }

      pcl::copyPointCloud(*cloud, output); // this copy is avoided by filtering "output" directly 

    }


    // function to apply bounding box to PointCloud with XYZRGB points
    void boundCloud(PointCloud &input, PointCloud &output, double box[]){

      PointCloud::Ptr cloud (new PointCloud);      // working copy for this routine
      for (int i=0; i<input.size(); i++) { // add points to cluster cloud
        cloud->push_back(input[i]);  
      } 

      std::cout<<"Beginning BoundCloud() function" << std::endl;
      //std::cout<<"Before bounding there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
      std::cout<<"Before bounding there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
      
      double box_length, box_width, box_height;
      box_length=0.25; // default auto_bounds, smart auto bounds not implemented
      box_width=0.25;     
      box_height=0.25;

      if (auto_bounds){
     
        Eigen::Vector4f centroid;
        Eigen::Vector4f min;
        Eigen::Vector4f max;  

        pcl::compute3DCentroid(*cloud, centroid);
        std::cout<<"The centroid of the points was found at: ["<<centroid[0]<<","<<centroid[1]<<","<<centroid[2]<<"]"<<std::endl; 

        box[0]=centroid[0]-box_length/2;  // xmin
        box[1]=centroid[0]+box_length/2;  // xmax
        box[2]=centroid[1]-box_width/2;   // ymin
        box[3]=centroid[1]+box_width/2;   // ymax
        box[4]=centroid[2]-box_height/2;  // zmin
        box[5]=centroid[2]+box_height/2;  // zmax

        std::cout<<"Using automatic bounding box limits: ["<<box[0]<<","<<box[1]<<","<<box[2]<<","<<box[3]<<","<<box[4]<<","<<box[5]<<"]"<< std::endl;
      }else{
        std::cout<<"Using bounding box limits: ["<<box[0]<<","<<box[1]<<","<<box[2]<<","<<box[3]<<","<<box[4]<<","<<box[5]<<"] from config file"<< std::endl;
      }

      //Apply Bounding Box Filter
      pcl::PassThrough<PointT> pass; //input_cloud
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
        
      std::cout<<"After bounding box filter there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
      // copy to the output cloud
      pcl::copyPointCloud(*cloud, output);

    }


    // function to apply translation and rotation without scaling to PointCloud
    void transformCloud(PointCloud &input, PointCloud &output, Eigen::Vector3f rotation, Eigen::Vector3f translation){

      PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy of the training cloud
      pcl::copyPointCloud(input,*cloud);

      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
     
      // Define a translation 
      transform.translation() << translation[0], translation[1], translation[2];
      // define three axis rotations (RPY)
      transform.rotate (Eigen::AngleAxisf (rotation[0], Eigen::Vector3f::UnitX()));
      transform.rotate (Eigen::AngleAxisf (rotation[1], Eigen::Vector3f::UnitY()));
      transform.rotate (Eigen::AngleAxisf (rotation[2], Eigen::Vector3f::UnitZ()));

      // Print the transformation
      //std::cout << transform_2.matrix() << std::endl;

      // Execute the transformation on working copy 
      pcl::transformPointCloud (*cloud, *cloud, transform); 
      // copy to the output cloud
      pcl::copyPointCloud(*cloud, output);
     
    }

    void smoothCloud(PointCloud &input, PointCloudNormal &output){

      PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy for this function 
      pcl::copyPointCloud(input,*cloud);

      // Create a KD-Tree
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

      // Output has the PointNormal type in order to store the normals calculated by MLS
      //pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points; // modify the function output pointcloud directly instead
      // Init object (second point type is for the normals, even if unused)
      pcl::MovingLeastSquares<PointT, pcl::PointXYZRGBNormal> mls;

      mls.setComputeNormals (true);
      // Set parameters

      mls.setInputCloud (cloud);
      mls.setPolynomialOrder (2);
      mls.setSearchMethod (tree);
      mls.setSearchRadius (0.03);

      // Reconstruct
      mls.process (output);

    }

    // templated function to perform PCL moving least squares smoothing, normal data is generated during this process
    template <typename point_t, typename point_normal_t> 
    void smoothCloudT(pcl::PointCloud<point_t> &input, pcl::PointCloud<point_normal_t> &output){

      typename pcl::PointCloud<point_t>::Ptr cloud (new pcl::PointCloud<point_t>);  //use this as the working copy for this function 
      pcl::copyPointCloud(input,*cloud);

      // Create a KD-Tree
      typename pcl::search::KdTree<point_t>::Ptr tree (new pcl::search::KdTree<point_t>);

      // Output has the PointNormal type in order to store the normals calculated by MLS
      //pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points; // modify the function output pointcloud directly instead
      // Init object (second point type is for the normals, even if unused)
      pcl::MovingLeastSquares<point_t, point_normal_t> mls;

      mls.setComputeNormals (true);
      // Set parameters

      mls.setInputCloud (cloud);
      //mls.setInputCloud (input);
      mls.setPolynomialOrder (2);
      mls.setSearchMethod (tree);
      mls.setSearchRadius (0.03);

      // Reconstruct
      mls.process (output);

    }

    
    // attributes

    // pointcloud pointers
    pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud;

    PointCloud *input, *downsampled, *transformed, *bounded, *smoothed; 
    
    // other parameters from the config file (these do not need to public)
    bool auto_bounds=0;
    bool save_output, translate_output, automatic_bounds, use_clustering, new_scan, transform_input;
    
    std::string package_path, input_path, output_path, input_file, output_file; 
   
    double bounding_box[6];
    Eigen::Vector3f pre_rotation, pre_translation;

    double voxel_size;

    // topic for generic cloud publisher
    std::string cloud_topic;


  private:

    // attributes

    // ros objects
    ros::NodeHandle node;
    ros::Rate rate;       // rate might be useful in 'public', but this is the proper place

    // generic publisher, can this be used for all of the clouds?
    //ros::Publisher cloud_pub = node.advertise<PointCloud> (cloud_topic, 1, true);
    std::vector<ros::Publisher> pub_clouds;
    std::vector<ros::Publisher> pub_clusters;
    int pub_idx;

    std::vector<ros::Publisher> pub_color, pub_euclidean, pub_intersection;

};


int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"filter_cloud");
  
  // instantiate an object filter.from the SeamDetection class
  FilterCloud filter;
 
  // load parameters from ROS param server, modify values in filter-cloud.yaml
  filter.loadConfig(); 
  
  std::cout<<"input_file: "<< filter.input_file << std::endl;

  // load the pointcloud from pcd file
  filter.loadCloud(filter.input_file, *filter.input);
  
  //voxel-downsampling, pre-transformation, and bounding-box on the input cloud
  filter.transformCloud(*filter.input, *filter.transformed, filter.pre_rotation, filter.pre_translation);
  filter.boundCloud(*filter.transformed, *filter.bounded, filter.bounding_box);
  filter.smoothCloudT(*filter.bounded, *filter.smoothed); 
  filter.downsampleCloud(*filter.smoothed, *filter.downsampled, filter.voxel_size); 
 
 
  // show the input training clouds in rviz
  filter.publishCloud(*filter.input, "/input"); 
  filter.publishCloud(*filter.downsampled, "/downsampled");
  filter.publishCloud(*filter.transformed, "/transformed"); 
  filter.publishCloud(*filter.bounded, "/bounded");
  filter.publishCloud(*filter.smoothed, "/smoothed");
  
  // save the cloud after processing 
  filter.saveCloud(filter.output_file, *filter.smoothed);

  std::cout<<"filter_cloud completed"<<std::endl;
  ros::spin();

  return 0;
}

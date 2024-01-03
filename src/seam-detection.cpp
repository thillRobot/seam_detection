/*
Seam Detection
This source contains the cpp class for this project
Created on 12/31/2023, Next year the project will be more organized!

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

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>

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

// PCL XYZ RGB Pointclouds
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

//typedef pcl::PointCloud< pcl::PointXYZRGB > PointCloud;
//typedef PointCloud::ConstPtr PointCloudConstPtr;
//typedef PointCloud::Ptr PointCloudPtr;

// aligned_allocator - STL compatible allocator to use with types requiring a non-standard alignment 
typedef std::vector < PointCloudPtr, Eigen::aligned_allocator < PointCloudPtr > > PointCloudVec;


class SeamDetection {

  public:

    // functions 
    
    // default constructor
    SeamDetection():rate(5) { // ROS::Rate rate(5) is in intializer list
      
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"|---------- SeamDetection v1.9 ----------|"<<std::endl;
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

      // allocate memory for pointclouds
      input_cloud = new PointCloud;
      transformed_cloud = new PointCloud;
      bounded_cloud = new PointCloud;

      recolored_cloud = new PointCloud;

      //cloud_color = new PointCloud; // not the best names, consider removing cloud_ prefix ...
      //cloud_euclidean = new PointCloud;

      // find the path to the this package (seam_detection)
      package_path = ros::package::getPath("seam_detection");
  
    }
  

    // function to load the config file(yaml) to pick the data files and set parameters 
    int LoadConfig(void){

      std::cout<<"|---------- SeamDetection::LoadConfig - loading configuration file ---------|"<<std::endl;

      // get boolen parameters 
      node.getParam("save_output", save_output);
      node.getParam("translate_output", translate_output);
      node.getParam("automatic_bounds", automatic_bounds);
      node.getParam("use_clustering", use_clustering);
      node.getParam("new_scan", new_scan);
      node.getParam("transform_input", transform_input);

      // get parameters that contain strings  
      node.getParam("seam_detection/input_file", input_file);
      node.getParam("seam_detection/output_file", output_file);
      node.getParam("seam_detection/target_file", target_file);
      
      // generate absolute file paths to inputs (does this belong here?)
      input_path=package_path+'/'+input_file;
      output_path=package_path+'/'+output_file;
      target_path=package_path+'/'+target_file;

      // get parameters that contain doubles 
      double voxel_leaf_size, cluster_tolerance;
      node.getParam("seam_detection/voxel_leaf_size", voxel_leaf_size);
      node.getParam("seam_detection/cluster_tolerance", cluster_tolerance);

      // get parameters that contain ints
      int cluster_min_size, cluster_max_size;
      node.getParam("seam_detection/cluster_min_size", cluster_min_size);
      node.getParam("seam_detection/cluster_max_size", cluster_max_size);

      // parameters that contain vectors of doubles
      std::vector<double> bounding_box_vec;
      //double filter_box[6];

      std::vector<double> pre_rotation_vec, pre_translation_vec;
      //Eigen::Vector3f pre_rotation, pre_translation;

      node.getParam("seam_detection/pre_rotation",  pre_rotation_vec);
      node.getParam("seam_detection/pre_translation",  pre_translation_vec);
      
      for(unsigned i=0; i < pre_rotation_vec.size(); i++){
        pre_rotation[i]=pre_rotation_vec[i]; // copy from std vector to eigen vector3f 
        pre_translation[i]=pre_translation_vec[i]; 
      }

      node.getParam("seam_detection/bounding_box",  bounding_box_vec);
      for(unsigned i=0; i < bounding_box_vec.size(); i++)
        bounding_box[i]=bounding_box_vec[i]; // copy from vector to array 
    
      return 0;
    }
    
    
    // function to load pointcloud from PCD file as defined in config
    int LoadCloud(std::string input_file){

      std::cout<<"|---------- SeamDetection::LoadCloud - loading configuration file ----------|"<<std::endl;
      
      node.getParam("seam_detection/input_file", input_file);
      input_path=package_path+'/'+input_file;

      // instantiate cloud pointer
      //PointCloud::Ptr input_cloud (new PointCloud); 
      PointCloud::Ptr cloud (new PointCloud);      // working copy for this routine

      std::cout << "Loading cloud input file:" << input_path << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_path, *input_cloud) == -1)
      {
          std::cout<<"Couldn't read cloud input file:"<<input_path;
          return (-1);
      }
      std::cout << "Loaded cloud input file: "<< input_path <<std::endl<<
        input_cloud->width * input_cloud->height << " Data points from "<< input_path << std::endl;
 
      return 0;  
    }


    // function to copy PointCloud with XYZRGB points - not needed, use pcl::copyPointCloud()
    void CopyCloud(PointCloud &input, PointCloud &output){

      std::cout<<"the point cloud input has "<< input.size()<< " points"<<std::endl;
      for (int i=0; i<input.size(); i++) { // add points to cluster cloud
        output.push_back(input[i]);  
      } 
      std::cout<<"the point cloud output has "<< output.size()<< " points"<<std::endl;
    
    }


    // function to apply bounding box to PointCloud with XYZRGB points
    void BoundCloud(PointCloud &input, PointCloud &output, double box[]){

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
    void TransformCloud(PointCloud &input, PointCloud &output, Eigen::Vector3f rotation, Eigen::Vector3f translation)
    {

      PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy of the target cloud
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

    // function to perform Euclidean clustering algorithm 
    void ExtractEuclideanClusters(PointCloud &input,  int min_size, int max_size, double tolerance)
    { 
      PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
      pcl::copyPointCloud(input,*cloud);

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      tree->setInputCloud (cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (tolerance); // cluster parameters set in config file
      ec.setMinClusterSize (min_size);
      ec.setMaxClusterSize (max_size);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud);
      ec.extract (cluster_indices);

      // instantiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
      //PointCloudVec clusters;

      int j = 0;
      for (const auto& cluster : cluster_indices) 
      {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) { // add points to cluster cloud
          cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        euclidean_clusters.push_back(cloud_cluster); // add clusters to vector of clusters
        j++; // increment the cluster counter
      }

      for (int i = 0; i < euclidean_clusters.size(); i++){
        std::cout << "euclidean cluster " << i << " has " << euclidean_clusters[i]->size() << " points " << std::endl;
      }

      std::cout<< "euclidean_clusters size: "<< euclidean_clusters.size() <<std::endl;

      //pcl::PointCloud <PointT>::Ptr colored = ec.getColoredCloud ();
      //pcl::copyPointCloud(*colored, *cloud_color); // copy to member attribute
        
      //return clusters;
    }


    void ExtractColorClusters(PointCloud &input, int min_size, double dist_thresh, double reg_color_thresh, double pt_color_thresh){
      PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy
      pcl::copyPointCloud(input,*cloud);

      // perform color based region growing segmentation  
      pcl::search::Search <PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

      pcl::IndicesPtr indices (new std::vector <int>);
      pcl::removeNaNFromPointCloud (*cloud, *indices);

      pcl::RegionGrowingRGB<PointT> reg;

      reg.setInputCloud (cloud);
      reg.setIndices (indices);
      reg.setSearchMethod (tree);
      reg.setDistanceThreshold (dist_thresh);
      reg.setPointColorThreshold (pt_color_thresh);
      reg.setRegionColorThreshold (reg_color_thresh);
      reg.setMinClusterSize (min_size);

      std::vector <pcl::PointIndices> cluster_indices;
      reg.extract(cluster_indices);

      pcl::PointCloud <PointT>::Ptr colored = reg.getColoredCloud ();
      pcl::copyPointCloud(*colored, *recolored_cloud); // copy to member attribute
      //this copy seems inefficient, fix later

      int j = 0;
      for (const auto& cluster : cluster_indices) 
      {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) { // add points to cluster cloud
          cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        color_clusters.push_back(cloud_cluster); // add clusters to vector of clusters
        j++; // increment the cluster counter
      }

      for (int i = 0; i < color_clusters.size(); i++){
        std::cout << "color cluster " << i << " has " << color_clusters[i]->size() << " points " << std::endl;
      }

      std::cout<< "color_clusters size: "<< color_clusters.size() <<std::endl;

      /*pcl::visualization::CloudViewer viewer ("Cluster viewer");
      viewer.showCloud (colored_cloud);
      //viewer.showCloud (color_clusters); // does not work, showCloud requires pointer

      while (!viewer.wasStopped ()){
        std::this_thread::sleep_for(100us); // 100us from std::chrono_literals
      }*/

    }
  

    // function to publish a single cloud (not working for multiple calls, blocking error)
    /*
    void PublishCloud(PointCloud &cloud, std::string topic)
    {

      std::cout<<"|---------- SeamDetection::PublishCloud - publishing single cloud ----------|"<<std::endl;

      ros::Publisher pub = node.advertise<PointCloud> (topic, 1, true);

      //input_cloud->header.frame_id = "base_link";
      cloud.header.frame_id = "base_link";

      pub.publish(cloud);
      ros::spin();

    }*/

    // function to publish the input and other clouds to ROS for RVIZ 
    void PublishClouds()
    {

      std::cout<<"|---------- SeamDetection::PublishClouds - publishing all clouds ----------|"<<std::endl;
 
      input_cloud->header.frame_id = "base_link";
      transformed_cloud->header.frame_id = "base_link";
      bounded_cloud->header.frame_id = "base_link";

      pub_input.publish(*input_cloud);
      pub_transformed.publish(*transformed_cloud);
      pub_bounded.publish(*bounded_cloud);
      
      ros::spinOnce();

    }

    // function to publish vectors of PointClouds from clustering 
    void PublishClusters()
    {
      
      std::cout<<"|---------- SeamDetection::PublishClusters - publishing clusters ----------|"<<std::endl;
      
      for (int i=0; i<euclidean_clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster from euclidean cluster extraction
        std::stringstream name;
        name << "euclidean_cluster" << i;
        pub_euclidean.push_back(node.advertise<PointCloud>(name.str(), 0, true));
        euclidean_clusters[i]->header.frame_id = "base_link";
        pub_euclidean[i].publish(euclidean_clusters[i]);
      }

      for (int i=0; i<color_clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster color based region growing cluster extraction
        std::stringstream name;
        name << "color_cluster" << i;
        pub_color.push_back(node.advertise<PointCloud>(name.str(), 0, true));
        color_clusters[i]->header.frame_id = "base_link";
        pub_color[i].publish(color_clusters[i]);
      }

      recolored_cloud->header.frame_id = "base_link";
      pub_recolored.publish(*recolored_cloud);
      
      ros::spinOnce();

    }

    // attributes
    PointCloud *input_cloud, *transformed_cloud, *bounded_cloud;
    PointCloud *recolored_cloud; // re-colored cloud from getColoredCloud() in color based extraction

    //PointCloudPtr bounded_cloud_ptr; 

    // vectors of pointclouds to store the separate clusters
    PointCloudVec color_clusters, euclidean_clusters;

    bool auto_bounds=0;
    bool save_output, translate_output, automatic_bounds, use_clustering, new_scan, transform_input;
    std::string package_path, input_path, output_path, target_path, input_file, output_file, target_file; 
   
    double bounding_box[6];
    Eigen::Vector3f pre_rotation, pre_translation;


  private:

    // attributes
    ros::NodeHandle node;
    ros::Rate rate;       // rate might be useful in 'public', but this is the proper place

    ros::Publisher pub_input = node.advertise<PointCloud> ("/input_cloud", 1, true);
    ros::Publisher pub_transformed = node.advertise<PointCloud> ("transformed_cloud", 1, true);
    ros::Publisher pub_bounded = node.advertise<PointCloud> ("bounded_cloud", 1, true);
    ros::Publisher pub_recolored = node.advertise<PointCloud> ("recolored_cloud", 1, true);

    std::vector<ros::Publisher> pub_color, pub_euclidean;

};


int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"seam_detection");
  
  SeamDetection sd;
 
  sd.LoadConfig();             // load parameters from config file to ros param server
  sd.LoadCloud(sd.input_file); // load a pointcloud from pcd file 

  PointCloud::Ptr cloud_copy (new PointCloud); // make copy here in main, just testing
  
  //sd.CopyCloud(*sd.input_cloud, *cloud_copy); // testing a useless function...
  pcl::copyPointCloud(*sd.input_cloud, *cloud_copy); // use the pcl copy

  sd.TransformCloud(*cloud_copy, *sd.transformed_cloud, sd.pre_rotation, sd.pre_translation);
  
  sd.BoundCloud(*sd.transformed_cloud, *sd.bounded_cloud, sd.bounding_box);

  //sd.PublishCloud(*sd.input_cloud, "/input_cloud"); // testing single cloud publish
  //sd.PublishCloud(*transformed_cloud, "/transformed_cloud"); // not working for multiple topics
  //sd.PublishCloud(*bounded_cloud, "/bounded_cloud");

  sd.ExtractEuclideanClusters(*sd.bounded_cloud, 200, 20000, 0.01); // preform euclidean cluster extraction

  sd.ExtractColorClusters(*sd.bounded_cloud, 200, 10, 6, 5); // preform color region growing cluster extraction

  sd.PublishClouds();  // show the input, transformed, and bounded clouds

  sd.PublishClusters(); // show the euclidean and color based clusters 
  
  ros::spin();

  return 0;
}

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

#include <pcl/common/common.h>
#include <pcl/pcl_config.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter_indices.h> 
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

#include <cloudfilter.h>
#include <cloudutils.h>


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

class SeamDetection {

  public:

    // functions 
    
    // default constructor
    SeamDetection(): rate(5), pub_idx(0) { // ROS::Rate rate(5) is in intializer list
      
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"|---------- SeamDetection v1.9 ----------|"<<std::endl;
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;
 
      // find the path to the this package (seam_detection)
      package_path = ros::package::getPath("seam_detection");
  
    }
  

    // function to load the config file(yaml) to pick the data files and set parameters 
    int loadConfig(void){

      std::cout<<"|---------- SeamDetection::LoadConfig - loading configuration file ---------|"<<std::endl;

      // get program control flags
      node.getParam("save_output", save_output);
      node.getParam("translate_output", translate_output);
      node.getParam("automatic_bounds", automatic_bounds);
      node.getParam("use_clustering", use_clustering);
      node.getParam("new_scan", new_scan);
      node.getParam("transform_input", transform_input);

      // get input file paths 
      node.getParam("seam_detection/training_file", training_file);
      node.getParam("seam_detection/test_file", test_file);    
      node.getParam("seam_detection/output_file", output_file);

      // file paths for multiview reconstruction
      node.getParam("training_view1_file", training_view1_file);
      node.getParam("training_view2_file", training_view2_file);
      node.getParam("training_view3_file", training_view3_file);
      node.getParam("training_view4_file", training_view4_file);
      node.getParam("training_merged_file", training_merged_file);
      node.getParam("seam_detection/training_inliers_file", training_inliers_file);
      node.getParam("training_merged_file", training_merged_file);
      
      node.getParam("test_view1_file", test_view1_file);
      node.getParam("test_view2_file", test_view2_file);
      node.getParam("test_view3_file", test_view3_file);
      node.getParam("test_view4_file", test_view4_file);
      node.getParam("test_merged_file", test_merged_file);
      node.getParam("seam_detection/test_inliers_file", test_inliers_file);
      node.getParam("test_merged_file", test_merged_file);

      // generate absolute file paths to inputs (does this belong here?)
      training_path=package_path+'/'+training_file; // i dont think so
      test_path=package_path+'/'+test_file;
      output_path=package_path+'/'+output_file;
      
      // get parameters that contain doubles 
      node.getParam("seam_detection/voxel_size", voxel_size);

      // parameters that contain vectors of doubles
      node.getParam("seam_detection/bounding_box",  bounding_box);

      // rotation and translation parameters from camera to fixed frame  
      node.getParam("seam_detection/pre_rotation",  pre_rotation);
      node.getParam("seam_detection/pre_translation",  pre_translation);

      // euclidean cluster extraction parameters
      node.getParam("euclidean_thresh", euclidean_thresh);
      node.getParam("euclidean_min_size", euclidean_min_size);
      node.getParam("euclidean_max_size", euclidean_max_size);
      node.getParam("euclidean_max_clusters", euclidean_max_clusters);
      
      // color based region growing segmentation parameters
      node.getParam("color_distance_thresh", color_distance_thresh);
      node.getParam("color_point_thresh", color_point_thresh);
      node.getParam("color_region_thresh", color_region_thresh);
      node.getParam("color_min_size", color_min_size);
      node.getParam("color_max_clusters", color_max_clusters);
          
      return 0;
    }
  
 
    // function to perform Euclidean Cluster Extraction  
    PointCloudVec extractEuclideanClusters(PointCloud &input){

      PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the training cloud
      pcl::copyPointCloud(input,*cloud);

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      tree->setInputCloud (cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (euclidean_thresh); // cluster parameters set in config file
      ec.setMinClusterSize (euclidean_min_size);
      ec.setMaxClusterSize (euclidean_max_size);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud);
      ec.extract (cluster_indices);

      // instantiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
      PointCloudVec clusters, clusters_out;

      int j = 0;
      for (const auto& cluster : cluster_indices) {

        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>); // allocate memory for the clouds in clusters
        for (const auto& idx : cluster.indices) { // add points to cluster cloud
          cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster); // add clusters to vector of clusters
        j++; // increment the cluster counter

      }

      // if there are fewer clusters than the max, the length will remain the same
      int n; // number of clouds in clusters_out
      if (clusters.size()<euclidean_max_clusters){
        n=clusters.size();
      }else{
        n=euclidean_max_clusters;
      }
      // put the first n clusters into clusters_out to be returned
      for (int i=0; i<n; i++){
        clusters_out.push_back(clusters[i]);
      }

      std::cout<< "euclidean clusters_out size: "<< clusters_out.size() <<std::endl;
      for (int i = 0; i < clusters_out.size(); i++){
        std::cout << "euclidean clusters_out " << i << " has " << clusters_out[i]->size() << " points " << std::endl;
      }
        
      return clusters_out;      
    }


    // function to compare cloud size, primarily for std::sort() used in extractColorClusters()
    static bool CompareSize(PointCloudPtr cloud_a, PointCloudPtr cloud_b){

      return cloud_a->size()>cloud_b->size(); // for sorting by greatest to least number of points in cloud

    }
 
    // overloaded function to compare cloud size for PointCloudNormal, primarily for std::sort() used in extractColorClusters()
    static bool CompareSizeNormal(PointCloudNormalPtr cloud_a, PointCloudNormalPtr cloud_b){

      return cloud_a->size()>cloud_b->size(); // for sorting by greatest to least number of points in cloud

    }


    // function to perform Color Based Region Growing Cluster Extraction 
    PointCloudVec extractColorClusters(PointCloud &input){
      
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
       
      reg.setDistanceThreshold (color_distance_thresh);
      reg.setPointColorThreshold (color_point_thresh);
      reg.setRegionColorThreshold (color_region_thresh);
      reg.setMinClusterSize (color_min_size);
  
      std::cout<<"distance threshold: "<<reg.getDistanceThreshold()<<std::endl;
      std::cout<<"point color threshold: "<<reg.getPointColorThreshold()<<std::endl;
      std::cout<<"region color threshold: "<<reg.getRegionColorThreshold()<<std::endl;

      std::vector <pcl::PointIndices> cluster_indices;
      reg.extract(cluster_indices);

      //pcl::PointCloud <PointT>::Ptr colored = reg.getColoredCloud ();
      //pcl::copyPointCloud(*colored, *recolored_cloud); // copy to member attribute
      // recolored cloud not used in workflow

      // instantiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
      PointCloudVec clusters, clusters_out;

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

        clusters.push_back(cloud_cluster); // add clusters to vector of clusters
        j++; // increment the cluster counter
        
      }

      // sort the cluster using user-defined compare function defined above 
      std::sort(clusters.begin(), clusters.end(), CompareSize);

      // if there are fewer clusters than the max, the length will remain the same
      int n; // number of clouds in clusters_out
      if (clusters.size()<color_max_clusters){
        n=clusters.size();
      }else{
        n=color_max_clusters;
      }
      // put the first n clusters into clusters_out to be returned
      for (int i=0; i<n; i++){
        clusters_out.push_back(clusters[i]);
      }

      std::cout<< "color clusters_out size: "<< clusters_out.size() <<std::endl;
      for (int i = 0; i < clusters_out.size(); i++){
        std::cout << "color clusters_out " << i << " has " << clusters_out[i]->size() << " points " << std::endl;
      }  

      return clusters_out;

    }
  

    //overloaded function to perform Color Based Region Growing Cluster Extraction and return PointCloudNormalVec
    PointCloudNormalVec extractColorClusters(PointCloudNormal &input){
      
      PointCloudNormal::Ptr cloud (new PointCloudNormal);       //use this as the working copy
      pcl::copyPointCloud(input,*cloud);

      // perform color based region growing segmentation  
      pcl::search::Search <PointNT>::Ptr tree (new pcl::search::KdTree<PointNT>);

      pcl::IndicesPtr indices (new std::vector <int>);
      pcl::removeNaNFromPointCloud (*cloud, *indices);

      pcl::RegionGrowingRGB<PointNT> reg;

      reg.setInputCloud (cloud);
      reg.setIndices (indices);
      reg.setSearchMethod (tree);
       
      reg.setDistanceThreshold (color_distance_thresh);
      reg.setPointColorThreshold (color_point_thresh);
      reg.setRegionColorThreshold (color_region_thresh);
      reg.setMinClusterSize (color_min_size);
  
      std::cout<<"distance threshold: "<<reg.getDistanceThreshold()<<std::endl;
      std::cout<<"point color threshold: "<<reg.getPointColorThreshold()<<std::endl;
      std::cout<<"region color threshold: "<<reg.getRegionColorThreshold()<<std::endl;

      std::vector <pcl::PointIndices> cluster_indices;
      reg.extract(cluster_indices);

      // instantiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
      PointCloudNormalVec clusters, clusters_out;

      int j = 0;
      for (const auto& cluster : cluster_indices) 
      {
        
        PointCloudNormal::Ptr cloud_cluster (new PointCloudNormal);
        for (const auto& idx : cluster.indices) { // add points to cluster cloud
          cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster); // add clusters to vector of clusters
        j++; // increment the cluster counter
        
      }

      // sort the cluster using user-defined compare function defined above 
      std::sort(clusters.begin(), clusters.end(), CompareSizeNormal);

      // if there are fewer clusters than the max, the length will remain the same
      int n; // number of clouds in clusters_out
      if (clusters.size()<color_max_clusters){
        n=clusters.size();
      }else{
        n=color_max_clusters;
      }
      // put the first n clusters into clusters_out to be returned
      for (int i=0; i<n; i++){
        clusters_out.push_back(clusters[i]);
      }

      std::cout<< "color clusters_out size: "<< clusters_out.size() <<std::endl;
      for (int i = 0; i < clusters_out.size(); i++){
        std::cout << "color clusters_out " << i << " has " << clusters_out[i]->size() << " points " << std::endl;
      }  

      return clusters_out;

    }


    //templated function to perform Color Based Region Growing Cluster Extraction and return PointCloudNormalVec
    template <typename point_t>
    std::vector < typename pcl::PointCloud<point_t>::Ptr, Eigen::aligned_allocator < typename pcl::PointCloud<point_t>::Ptr> > 
    extractColorClustersT(pcl::PointCloud<point_t> &input){
      
      typename pcl::PointCloud<point_t>::Ptr cloud (new pcl::PointCloud<point_t>);       //use this as the working copy
      pcl::copyPointCloud(input,*cloud);

      // perform color based region growing segmentation  
      typename pcl::search::Search <point_t>::Ptr tree (new pcl::search::KdTree<point_t>);

      pcl::IndicesPtr indices (new std::vector <int>);
      pcl::removeNaNFromPointCloud (*cloud, *indices);

      pcl::RegionGrowingRGB<point_t> reg;

      reg.setInputCloud (cloud);
      reg.setIndices (indices);
      reg.setSearchMethod (tree);
       
      reg.setDistanceThreshold (color_distance_thresh);
      reg.setPointColorThreshold (color_point_thresh);
      reg.setRegionColorThreshold (color_region_thresh);
      reg.setMinClusterSize (color_min_size);
  
      std::cout<<"distance threshold: "<<reg.getDistanceThreshold()<<std::endl;
      std::cout<<"point color threshold: "<<reg.getPointColorThreshold()<<std::endl;
      std::cout<<"region color threshold: "<<reg.getRegionColorThreshold()<<std::endl;

      std::vector <pcl::PointIndices> cluster_indices;
      reg.extract(cluster_indices);

      // instantiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
      //PointCloudNormalVec clusters, clusters_out;
      std::vector < typename pcl::PointCloud<point_t>::Ptr, 
                    Eigen::aligned_allocator < typename pcl::PointCloud<point_t>::Ptr> > clusters, clusters_out;

      int j = 0;
      for (const auto& cluster : cluster_indices) 
      {
        
        typename pcl::PointCloud<point_t>::Ptr cloud_cluster (new pcl::PointCloud<point_t>);
        for (const auto& idx : cluster.indices) { // add points to cluster cloud
          cloud_cluster->push_back((*cloud)[idx]);
        } 
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster); // add clusters to vector of clusters
        j++; // increment the cluster counter
        
      }

      // sort the cluster using user-defined compare function defined above 
      std::sort(clusters.begin(), clusters.end(), CompareSizeNormal);

      // if there are fewer clusters than the max, the length will remain the same
      int n; // number of clouds in clusters_out
      if (clusters.size()<color_max_clusters){
        n=clusters.size();
      }else{
        n=color_max_clusters;
      }
      // put the first n clusters into clusters_out to be returned
      for (int i=0; i<n; i++){
        clusters_out.push_back(clusters[i]);
      }

      std::cout<< "color clusters_out size: "<< clusters_out.size() <<std::endl;
      for (int i = 0; i < clusters_out.size(); i++){
        std::cout << "color clusters_out " << i << " has " << clusters_out[i]->size() << " points " << std::endl;
      }  

      return clusters_out;

    }


    // function to find the minimum oriented bounding box of a cloud using principle component analysis
    void getPCABox(PointCloud &input, Eigen::Quaternionf& rotation, Eigen::Vector3f& translation, Eigen::Vector3f& dimension){

      PointCloud::Ptr cloud (new PointCloud); //allocate memory 
      pcl::copyPointCloud(input,*cloud);      //and make working copy of the input cloud 

      // Compute principal directions
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud, centroid);
      Eigen::Matrix3f covariance;
      computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
      eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

      // Transform the original cloud to the origin where the principal components correspond to the axes.
      Eigen::Matrix4f projection_transform(Eigen::Matrix4f::Identity());
      projection_transform.block<3,3>(0,0) = eigen_vectors.transpose();
      projection_transform.block<3,1>(0,3) = -1.f * (projection_transform.block<3,3>(0,0) * centroid.head<3>());
      PointCloudPtr projected_cloud (new PointCloud);
      
      pcl::transformPointCloud(*cloud, *projected_cloud, projection_transform);
      // Get the minimum and maximum points of the transformed cloud.
      PointT min_point, max_point;
      pcl::getMinMax3D(*projected_cloud, min_point, max_point);
      const Eigen::Vector3f mean_diagonal = 0.5f*(max_point.getVector3fMap() + min_point.getVector3fMap());

      // Final transform
      const Eigen::Quaternionf box_rotation(eigen_vectors); 
      const Eigen::Vector3f box_translation = eigen_vectors * mean_diagonal + centroid.head<3>();
      
      dimension[0]=max_point.x-min_point.x; // store the x,y,z lengths of the bounding box
      dimension[1]=max_point.y-min_point.y;
      dimension[2]=max_point.z-min_point.z;

      double volume, aspect_ratio;
      volume=dimension[0]*dimension[1]*dimension[2]; // calculate volume as product of dimensions
      aspect_ratio=dimension.maxCoeff()/dimension.minCoeff(); // calculate aspect ratio as max dimension / min dimension

      rotation=box_rotation;    // copy to the output variables, these lines crash now that I am passing in a vector ...
      translation=box_translation;

    }

     
    // overloaded function to find minimum oriented bounding box of cloud using principle component analysis,
    // gives access to eigen_vectors for debugging
    void getPCABox(PointCloud &input, 
                   Eigen::Quaternionf& rotation, 
                   Eigen::Vector3f& translation, 
                   Eigen::Vector3f& dimension, 
                   Eigen::Matrix3f eigen_vectors){

      PointCloud::Ptr cloud (new PointCloud); //allocate memory 
      pcl::copyPointCloud(input,*cloud);      //and make working copy of the input cloud 

      // Compute principal directions
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud, centroid);
      Eigen::Matrix3f covariance;
      computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      //Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
      eigen_vectors = eigen_solver.eigenvectors();
      eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

      // Transform the original cloud to the origin where the principal components correspond to the axes.
      Eigen::Matrix4f projection_transform(Eigen::Matrix4f::Identity());
      projection_transform.block<3,3>(0,0) = eigen_vectors.transpose();
      projection_transform.block<3,1>(0,3) = -1.f * (projection_transform.block<3,3>(0,0) * centroid.head<3>());
      PointCloudPtr projected_cloud (new PointCloud);
      
      pcl::transformPointCloud(*cloud, *projected_cloud, projection_transform);
      // Get the minimum and maximum points of the transformed cloud.
      PointT min_point, max_point;
      pcl::getMinMax3D(*projected_cloud, min_point, max_point);
      const Eigen::Vector3f mean_diagonal = 0.5f*(max_point.getVector3fMap() + min_point.getVector3fMap());

      // Final transform
      const Eigen::Quaternionf box_rotation(eigen_vectors); 
      const Eigen::Vector3f box_translation = eigen_vectors * mean_diagonal + centroid.head<3>();
      
      dimension[0]=max_point.x-min_point.x; // store the x,y,z lengths of the bounding box
      dimension[1]=max_point.y-min_point.y;
      dimension[2]=max_point.z-min_point.z;


      double volume, aspect_ratio;
      volume=dimension[0]*dimension[1]*dimension[2]; // calculate volume as product of dimensions
      aspect_ratio=dimension.maxCoeff()/dimension.minCoeff(); // calculate aspect ratio as max dimension / min dimension

      rotation=box_rotation;    // copy to the output variables, these lines crash now that I am passing in a vector ...
      translation=box_translation;

    }


    //function to get principle component axis boxes for a vector of pointclouds, calls SeamDetection::getPCABox()
    void getPCABoxes(PointCloudVec &clouds){

      std::vector < Eigen::Quaternionf > quaternions; // vector of quaternions, maybe not the best solution...
      std::vector < Eigen::Vector3f > translations;   // these could be in a 2D array, but then the syntax would not match
      std::vector < Eigen::Vector3f > dimensions;

      for (int i = 0; i < clouds.size(); i++){
        
        // get the pose and size of the minimum bounding box for each cluster
        /// this is a temp variable to get the eigen::quaternion from the function which will be added to quaternions vector 
        Eigen::Quaternionf quaternion; 
        Eigen::Vector3f translation, dimension; // these are also temp vars for the same purpose,  
        getPCABox(*clouds[i], quaternion, translation, dimension);  // does not work (compiles but throws runtime error), check this 
        
        // add the temp variable to the vectors
        quaternions.push_back(quaternion);  
        translations.push_back(translation); 
        dimensions.push_back(dimension); 

        // print cluster information to the terminal for debug
        std::cout << "cluster "<< i <<" number of points: " << clouds[i]->size() << std::endl;
        std::cout << "cluster "<< i <<" PCA box quaternion: ["<< quaternions[i].x()<<","
                                            << quaternions[i].y()<<","
                                            << quaternions[i].z()<<","
                                            << quaternions[i].w()<<"]"<<std::endl;                                    
        std::cout << "cluster "<< i <<" PCA box translation: ["<< translations[i][0] << "," 
                                            <<translations[i][1] << "," 
                                            <<translations[i][2] << "]" <<std::endl;
        std::cout << "cluster "<< i <<" PCA box dimension: ["<< dimensions[i][0] << "," 
                                            <<dimensions[i][1] << "," 
                                            <<dimensions[i][2] << "]" <<std::endl;
      }
    }

 
    // function to calculate the the objection function value or score for a pair of PointClouds
    // to be used to find agreement between potentially overlapping clouds 
    template <typename point_t>
    double scoreClouds(pcl::PointCloud<point_t> &cloud1, pcl::PointCloud<point_t> &cloud2, int verbosity){
          
      std::cout<<"|---------- SeamDetection::scoreClouds() ----------|"<<std::endl;
      
      double score=100, f1, f2, f3, f4, 
             distance_x, distance_y, distance_z,
             cloud_volume, compare_volume, 
             cloud_aspect_ratio, compare_aspect_ratio,
             difference_x, difference_y, difference_z;

      // find the pca min bounding box for the first cloud
      Eigen::Quaternionf cloud_quaternion; 
      Eigen::Vector3f cloud_translation, cloud_size;
      Eigen::Matrix3f cloud_eigenvectors;
      getPCABox(cloud1, cloud_quaternion, cloud_translation, cloud_size, cloud_eigenvectors);

      // find the pca min bounding box for the second cloud 
      Eigen::Quaternionf compare_quaternion; 
      Eigen::Vector3f compare_translation, compare_size;
      Eigen::Matrix3f compare_eigenvectors;  
      getPCABox(cloud2, compare_quaternion, compare_translation, compare_size, compare_eigenvectors);

      // calculate separate terms for the objective function between the two clouds

      // term1 - position of centroid
      distance_x=cloud_translation[0]-compare_translation[0];
      distance_y=cloud_translation[1]-compare_translation[1];
      distance_z=cloud_translation[2]-compare_translation[2]; 
      // square root of sum of squared component distances between centroids - l
      f1 = pow(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2), 1.0/2.0);
      //f1 = 0; // disabled temporarily   
      //std::cout<<"f1: "<<f1<<std::endl;

      // term2 - volume of bounding box
      cloud_volume=cloud_size[0]*cloud_size[1]*cloud_size[2];
      compare_volume=compare_size[0]*compare_size[1]*compare_size[2];
      f2 = pow(std::abs(cloud_volume-compare_volume),1.0/3.0); // cube root of difference in volume - l
      f2 = std::abs(compare_volume-compare_volume);               
      //std::cout<<"f2: "<<f2<<std::endl;

      // term3 - aspect ratio of bounding box 
      cloud_aspect_ratio=  cloud_size.maxCoeff()/cloud_size.minCoeff(); 
      compare_aspect_ratio=  compare_size.maxCoeff()/compare_size.minCoeff(); 
      f3 = pow(pow(cloud_aspect_ratio - compare_aspect_ratio, 2), 1.0/2.0); // square root of squared difference in aspect ratios - l
      //f3 = 0; // disabled temporarily
      //std::cout<<"f3: "<<f3<<std::endl;

      // term4 - orientation of bounding box
      //difference_x=cloud_size[0]-compare_size[0]; // this does not seem right, does not contain orientation info...
      //difference_y=cloud_size[1]-compare_size[1]; // need to use projection onto fixed framed
      //difference_z=cloud_size[2]-compare_size[2]; 
      // square root of sum of square dimension differences - l
      //f4 = pow(pow(difference_x,2)+pow(difference_y,2)+pow(difference_z,2), 1.0/2.0); 
      f4 = 0; // disabled temporarily 
      //std::cout<<"f4: "<<f4<<std::endl;
      
      // term4 - color metric
      
      // objective function value is sum of terms 
      score=f1+f2+f3+f4;
      
      if(verbosity>1){
        std::cout<<"cloud_translation: "<<std::endl<<"["<<cloud_translation[0]<<","
                                                        <<cloud_translation[1]<<","
                                                        <<cloud_translation[2]<<"]"<<std::endl;
        std::cout<<"compare_translation: "<<std::endl<<"["<<compare_translation[0]<<","
                                                          <<compare_translation[1]<<","
                                                          <<compare_translation[2]<<"]"<<std::endl;

        std::cout<<"cloud_size: "<<std::endl<<"["<<cloud_size[0]<<","
                                                 <<cloud_size[1]<<","
                                                 <<cloud_size[2]<<"]"<<std::endl;
        std::cout<<"cloud_volume: "<<cloud_volume<<std::endl;
        std::cout<<"compare_size: "<<std::endl<<"["<<compare_size[0]<<","
                                                   <<compare_size[1]<<","
                                                   <<compare_size[2]<<"]"<<std::endl;
        std::cout<<"compare_volume: "<<compare_volume<<std::endl;

        std::cout<<"cloud_eigenvectors: "<<std::endl<<"[" << cloud_eigenvectors(0,0)<<","
                                                          << cloud_eigenvectors(0,1)<<","
                                                          << cloud_eigenvectors(0,2)<<std::endl
                                                          << cloud_eigenvectors(1,0)<<","
                                                          << cloud_eigenvectors(1,1)<<","
                                                          << cloud_eigenvectors(1,2)<<std::endl
                                                          << cloud_eigenvectors(2,0)<<","
                                                          << cloud_eigenvectors(2,1)<<","
                                                          << cloud_eigenvectors(2,2)<<"]"<<std::endl;
        
        std::cout<<"compare_eigenvectors: "<<std::endl<<"[" << compare_eigenvectors(0,0)<<","
                                                            << compare_eigenvectors(0,1)<<","
                                                            << compare_eigenvectors(0,2)<<std::endl
                                                            << compare_eigenvectors(1,0)<<","
                                                            << compare_eigenvectors(1,1)<<","
                                                            << compare_eigenvectors(1,2)<<std::endl
                                                            << compare_eigenvectors(2,0)<<","
                                                            << compare_eigenvectors(2,1)<<","
                                                            << compare_eigenvectors(2,2)<<"]"<<std::endl;
        
        std::cout<<"objective function value: "<<score<<std::endl;                              
      }
      
      // return objective function value 'score' as the sum of the terms
      return score=f1+f2+f3+f4;
    }
    

    // function to find best 1 to 1 correlation between two sets of clusters using scoreCloud()
    // for now this assumes size of clusters is less than or equal to size of compares to ensure 1-1 correspondence 
    PointCloudVec matchClusters(PointCloudVec clusters, PointCloudVec compares, int verbosity){

      double score, score_min;
      int j_min, success;

      PointCloudVec matches;
      matches=clusters; // make a copy to get the size, fix this soon

      if (clusters.size()<=compares.size()){         // clusters has fewer clusters than compares
         
        for (int i=0; i<clusters.size(); i++){               // for each cluster in clusters find best match from compares 
          
          score_min=scoreClouds(*clusters[0], *compares[0], verbosity);  // seed the search with the score of first pair 
          j_min=0;                                              // dont forget to initialize the search index  ! 

          for (int j=0; j<compares.size(); j++){
            score=scoreClouds(*clusters[i], *compares[j], verbosity);

            if (score<score_min){
              score_min=score;    // store the min score
              j_min=j;            // and the index of the min
            }

            if(verbosity>1){  // show the pairs score as the search is performed 
              std::cout <<"clusters["<<i<<"] (size:" <<clusters[i]->size()<<") and compares["<<j
                        <<"] (size:"<<compares[j]->size()<<") have a score "<<score<<std::endl<<std::endl;
            }  
      
          }

          // after checking all potential matches, push the best match into the vector of matches with the recorded index
          matches.at(i)=compares[j_min];
          compares.erase(compares.begin()+j_min); // remove the match from the set of compares for 1-1 correspondence 

        }
        
        if (verbosity>0){  
          for (int k=0; k<clusters.size(); k++){ // show the results after the search is complete
            std::cout<<"cluster["<<k<<"] has "<< clusters[k]->size()<< " points " 
            <<" and matches["<<k<<"] has "<<matches[k]->size()<< " points"<<std::endl;
          }
        }

      }else{       // compares has fewer clusters than clusters, empty return 
        std::cout<<"warning: ( clusters.size() <= compares.size() ) failed, no matches returned"<<std::endl;
      }

      return matches;
    }

    // function to return the objective function value (score) for a pointcloud vs. each pointcloud in compares
    // this uses a media normalized (scaled) multi objective function
    Eigen::VectorXd scoreCloudsMulti(PointCloud &cloud, PointCloudVec compares, float use_centroid){
      
      std::cout<<"|---------- SeamDetection::scoreCloudsMulti() ----------|"<<std::endl;
    
      Eigen::VectorXd centroid_diffs(compares.size()), // vectors of differences, using eigen for vectorized ops (maybe)
                      volume_diffs(compares.size()),   // size might not be required as these are Xd 
                      aspect_ratio_diffs(compares.size()),
                      centroid_diffs_norm(compares.size()), // normalized vectors of differences 
                      volume_diffs_norm(compares.size()), 
                      aspect_ratio_diffs_norm(compares.size()),
                      med_red_diffs(compares.size()),
                      med_red_diffs_norm(compares.size()),
                      med_green_diffs(compares.size()),
                      med_green_diffs_norm(compares.size()),
                      med_blue_diffs(compares.size()), 
                      med_blue_diffs_norm(compares.size()),
                      scores(compares.size());  // vector of objective function values (scores)      
                      
      Eigen::VectorXd cloud_med_rgb, compare_med_rgb;
      Eigen::MatrixXd med_rgb_diffs(3,compares.size());               

      double  distance_x, distance_y, distance_z, 
              cloud_volume, compare_volume, cloud_aspect_ratio, compare_aspect_ratio,
              difference_x, difference_y, difference_z,
              centroid_diffs_median, volume_diffs_median, aspect_ratio_diffs_median,
              med_red_diffs_median, med_green_diffs_median, med_blue_diffs_median,
              score, score_min;

      int n, j_min;
      
      Eigen::Quaternionf cloud_quaternion; 
      Eigen::Vector3f cloud_translation, cloud_size;
      Eigen::Matrix3f cloud_eigenvectors;

      Eigen::Quaternionf compare_quaternion; 
      Eigen::Vector3f compare_translation, compare_size;
      Eigen::Matrix3f compare_eigenvectors;                      
     
      CloudUtils utl;
      // find the pca min bounding box for the ith cloud in clusters, only required to run once for single cloud 
      getPCABox(cloud, cloud_quaternion, cloud_translation, cloud_size, cloud_eigenvectors);

      // compare cloud to each cluster in compares                           
      for (int j=0; j<compares.size(); j++){

        // find the pca min bounding box for the jth cloud in compares
        getPCABox(*compares[j], compare_quaternion, compare_translation, compare_size, compare_eigenvectors);

        // calculate the objective differences for each pair 
        // term1 - position of centroid
        distance_x=cloud_translation[0]-compare_translation[0];
        distance_y=cloud_translation[1]-compare_translation[1];
        distance_z=cloud_translation[2]-compare_translation[2]; 
        // square root of sum of squared component distances between centroids - l
        centroid_diffs[j] = pow(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2), 1.0/2.0);
        
        // term2 - volume of bounding box
        cloud_volume=cloud_size[0]*cloud_size[1]*cloud_size[2];
        compare_volume=compare_size[0]*compare_size[1]*compare_size[2];
        volume_diffs[j]=std::abs(cloud_volume-compare_volume);
        
        // term3 - aspect ratio of bounding box 
        cloud_aspect_ratio=  cloud_size.maxCoeff()/cloud_size.minCoeff(); 
        compare_aspect_ratio=  compare_size.maxCoeff()/compare_size.minCoeff(); 
        aspect_ratio_diffs[j]= std::abs(cloud_aspect_ratio - compare_aspect_ratio); 

        // term4 - color metric
        //std::cout<<"calculating term 4 - color metric" <<std::endl;
        cloud_med_rgb=utl.getMedianColor(cloud);
        compare_med_rgb=utl.getMedianColor(*compares[j]);
                  
        med_red_diffs[j]=std::abs(cloud_med_rgb[0]-compare_med_rgb[0]); 
        med_green_diffs[j]=std::abs(cloud_med_rgb[1]-compare_med_rgb[1]);
        med_blue_diffs[j]=std::abs(cloud_med_rgb[2]-compare_med_rgb[2]);
        
        //scoreClouds(cloud, compares[j]); // ideally this would call scoreClouds() to keep obj fun in one place

      }

      // find the median value for each objective 
      centroid_diffs_median=utl.getMedian(centroid_diffs);
      volume_diffs_median=utl.getMedian(volume_diffs);
      aspect_ratio_diffs_median=utl.getMedian(aspect_ratio_diffs);
      med_red_diffs_median=utl.getMedian(med_red_diffs);
      med_green_diffs_median=utl.getMedian(med_green_diffs);
      med_blue_diffs_median=utl.getMedian(med_blue_diffs);
      
      // normalize diffs by dividing by median difference for each objective 
      centroid_diffs_norm=centroid_diffs/centroid_diffs_median; // use vectorized row operations from library Eigen
      volume_diffs_norm=volume_diffs/volume_diffs_median;       
      aspect_ratio_diffs_norm=aspect_ratio_diffs/aspect_ratio_diffs_median;
      med_red_diffs_norm=med_red_diffs/med_red_diffs_median;
      med_green_diffs_norm=med_green_diffs/med_green_diffs_median;
      med_blue_diffs_norm=med_blue_diffs/med_blue_diffs_median;

      //scores=centroid_diffs_norm+volume_diffs_norm+aspect_ratio_diffs_norm;
      // return the score as the sum of the normalized terms for each pair   
      return centroid_diffs_norm*use_centroid+volume_diffs_norm+aspect_ratio_diffs_norm
             +med_red_diffs_norm+med_green_diffs_norm+med_blue_diffs_norm; 
      
    }

   
    // function to find best match between sets of clusters using multi-objective optimization
    // used to correlate sets of clusters from different algorithms 
    PointCloudVec matchClustersMulti(PointCloudVec clusters, PointCloudVec compares, int verbosity, float use_centroid){
      
      std::cout<<"|---------- SeamDetection::matchClustersMulti() ----------|"<<std::endl;
       
      PointCloudVec matches;
      matches=clusters; // make a copy just to get the size, change to an empty copy later

      double  score, score_min;

      int n, j_min;
      
      CloudUtils utl;
      
      if (clusters.size()<=compares.size()){  // clusters has fewer clusters than compares
        n=clusters.size();
      }else{
        n=compares.size();
        std::cout<<"( clusters.size() <= compares.size() ) failed, matching clusters[1:compares.size()] to compares[:]"<<std::endl; 
      } 
      
      Eigen::VectorXd scores;

      for (int i=0; i<n; i++){  // compare each cluster in clusters to each cluster in compares 
          
        scores=scoreCloudsMulti(*clusters[i], compares, use_centroid); // get the scores for clusters[i] and all compares        

  
        // find pair with min sum objective difference using median normalized differences 
        j_min=0; // default value for the search index, in case it is not set
        
        // seed the minimization with the first score in the set
        score_min=scores[j_min];        

        // search for lowest score in vector of scores found previously        
        for(int j=0; j<scores.size(); j++){ // re-check each possible pair
          std::cout<<"scores["<<j<<"]:"<<scores[j]<<std::endl;
          if (scores[j]<score_min){ // find the lowest score
            score_min=scores[j];
            j_min=j;            // record the index of the lowest score

          }
        }          
        std::cout<<"scores_min: "<<score_min<<" found at "<<j_min<<std::endl;
        // add the compare with the best score to matches
        matches.at(i)=compares[j_min];
        // remove the match from the compare set for next iteration
        compares.erase(compares.begin()+j_min);

        if(verbosity>1){ // show the values as the search is performed
          std::cout<<std::endl<<"iteration "<<i<<std::endl;
        }
        
      }  
      
      if(verbosity>0){ // show the results of the search after complete
        for (int k=0; k<clusters.size(); k++){
          std::cout <<"cluster["<<k<<"] has "<< clusters[k]->size()<< " points " 
                    <<" and matches["<<k<<"] has "<<matches[k]->size()<< " points"<<std::endl;
        }             
      }
      return matches;
    }// end of matchClustersMulti() function
    

    // overloaded function to find best match between single pointcloud and set of clusters using multi-objective optimization
    PointCloudPtr matchCloudToClustersMulti(PointCloud &cloud, PointCloudVec compares, int verbosity, float use_centroid){

      Eigen::VectorXd scores;
      scores=scoreCloudsMulti(cloud, compares, use_centroid);  

      // find pair with min sum objective difference using median normalized differences, replace with std::min for speed
      int j_min;
      double score_min;
      verbosity=2;
      j_min=0; // default value for the search index, in case it is not set
      score_min=scores[0]; // assume first may be the minimum

      for(int j=0; j<compares.size(); j++){ // re-check each possible pair
        if (scores[j]<score_min){ // find the lowest score
          score_min=scores[j];
          j_min=j;            // record the index of the lowest score
        }
        std::cout<<"matching score ["<<j<<"]: "<<scores[j]<<std::endl;
      }          

      // printing for debugging, optional, controlled by fn arg
      if(verbosity>1){ // show the diff values from the search each iteration

      }else if(verbosity>0){ // show the results of the search after complete
        std::cout <<"cloud has "<< cloud.size()<< " points " 
                  <<" and match has "<<compares[j_min]->size()<< " points "<<std::endl;
                   
      }
      
      return compares[j_min];  // return the compare with the best score to matches
    }

    
    // function to return the objective function value (score) for a pointcloud vs. each pointcloud 
    // in compares1 and compares2, this uses a media normalized (scaled) multi objective function
    // this seems uneccesary, who thought of this anyway?!?!
    Eigen::VectorXd scoreCloudsMulti2(PointCloud &cloud, PointCloudVec compares1, PointCloudVec compares2){

      Eigen::VectorXd centroid_diffs1(compares1.size()), // vectors of differences, using eigen for vectorized ops (maybe)
                      volume_diffs1(compares1.size()),   // size might not be required as these are Xd 
                      aspect_ratio_diffs1(compares1.size()),
                      centroid_diffs1_norm(compares1.size()), // normalized vectors of differences 
                      volume_diffs1_norm(compares1.size()), 
                      aspect_ratio_diffs1_norm(compares1.size());
                      
      Eigen::VectorXd centroid_diffs2(compares1.size()), 
                      volume_diffs2(compares1.size()),    
                      aspect_ratio_diffs2(compares1.size()),
                      centroid_diffs2_norm(compares1.size()), 
                      volume_diffs2_norm(compares1.size()), 
                      aspect_ratio_diffs2_norm(compares1.size());
                      
      Eigen::VectorXd scores(compares1.size());  // vector of objective function values (scores)      
                    
      double  distance_x, distance_y, distance_z, 
              cloud_volume, compare_volume, cloud_aspect_ratio, compare_aspect_ratio,
              difference_x, difference_y, difference_z,
              centroid_diffs_median, volume_diffs_median, aspect_ratio_diffs_median;
      double  score, score_min;

      int n, j_min;
      
      Eigen::Quaternionf cloud_quaternion; 
      Eigen::Vector3f cloud_translation, cloud_size;
      Eigen::Matrix3f cloud_eigenvectors;

      Eigen::Quaternionf compare1_quaternion; 
      Eigen::Vector3f compare1_translation, compare1_size;
      Eigen::Matrix3f compare1_eigenvectors;  

      Eigen::Quaternionf compare2_quaternion; 
      Eigen::Vector3f compare2_translation, compare2_size;
      Eigen::Matrix3f compare2_eigenvectors;                     
     
      // find the pca min bounding box for the ith cloud in clusters, only required to run once for single cloud 
      getPCABox(cloud, cloud_quaternion, cloud_translation, cloud_size, cloud_eigenvectors);

      // compare cloud to each cluster in compares                           
      for (int j=0; j<compares1.size(); j++){

        // find the pca min bounding box for the jth cloud in compares1
        getPCABox(*compares1[j], compare1_quaternion, compare1_translation, compare1_size, compare1_eigenvectors);

        // find the pca min bounding box for the jth cloud in compares2
        getPCABox(*compares2[j], compare2_quaternion, compare2_translation, compare2_size, compare2_eigenvectors);

        std::cout<<"DEBUG: returned from getPCABox function twice"<<std::endl;

        // calculate the objective differences between cloud and compares1
        // term1,1 - position of centroid differnce from compare1
        distance_x=cloud_translation[0]-compare1_translation[0];
        distance_y=cloud_translation[1]-compare1_translation[1];
        distance_z=cloud_translation[2]-compare1_translation[2];
        // square root of sum of squared component distances between centroids - l
        centroid_diffs1[j] = pow(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2), 1.0/2.0); 
        
        // term2,1 - volume of bounding box differnce from compare1
        cloud_volume=cloud_size[0]*cloud_size[1]*cloud_size[2];
        compare_volume=compare1_size[0]*compare1_size[1]*compare1_size[2];
        volume_diffs1[j]=std::abs(cloud_volume-compare_volume);
        
        // term3,1 - aspect ratio of bounding box difference from compare1
        cloud_aspect_ratio=  cloud_size.maxCoeff()/cloud_size.minCoeff(); 
        compare_aspect_ratio=  compare1_size.maxCoeff()/compare1_size.minCoeff();  
        // square root of squared difference in aspect ratios - l
        aspect_ratio_diffs1[j]= std::abs(cloud_aspect_ratio - compare_aspect_ratio);

        // calculate the objective differences between cloud and compares2
        // term1,2 - position of centroid difference from compare2
        distance_x=cloud_translation[0]-compare2_translation[0];
        distance_y=cloud_translation[1]-compare2_translation[1];
        distance_z=cloud_translation[2]-compare2_translation[2];
        // square root of sum of squared component distances between centroids - l
        centroid_diffs2[j] = pow(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2), 1.0/2.0); 
        
        // term2,2 - volume of bounding box difference from compare2
        cloud_volume=cloud_size[0]*cloud_size[1]*cloud_size[2];
        compare_volume=compare2_size[0]*compare2_size[1]*compare2_size[2];
        volume_diffs2[j]=std::abs(cloud_volume-compare_volume);
        
        // term3,2 - aspect ratio of bounding box difference from compare2
        cloud_aspect_ratio=  cloud_size.maxCoeff()/cloud_size.minCoeff(); 
        compare_aspect_ratio=  compare2_size.maxCoeff()/compare2_size.minCoeff(); 
        aspect_ratio_diffs2[j]= std::abs(cloud_aspect_ratio - compare_aspect_ratio);

      }

      std::cout<<"DEBUG: finished calculating diffs"<<std::endl;
      
      CloudUtils utl;

      // find the median value for each objective from cloud and compare1
      centroid_diffs_median=utl.getMedian(centroid_diffs1);
      volume_diffs_median=utl.getMedian(volume_diffs1);
      aspect_ratio_diffs_median=utl.getMedian(aspect_ratio_diffs1);
      
      // normalize diffs by dividing by median difference for each objective from cloud and compare 1
      centroid_diffs1_norm=centroid_diffs1/centroid_diffs_median; // use vectorized row operations from library Eigen
      volume_diffs1_norm=volume_diffs1/volume_diffs_median;       
      aspect_ratio_diffs1_norm=aspect_ratio_diffs1/aspect_ratio_diffs_median;

      // normalize diffs by dividing by median difference for each objective from cloud and compare 1
      //centroid_diffs1_norm=centroid_diffs1/getMedian(centroid_diffs1); // use vectorized row operations from library Eigen
      //volume_diffs1_norm=volume_diffs1/getMedian(volume_diffs1);      
      //aspect_ratio_diffs1_norm=aspect_ratio_diffs1/getMedian(aspect_ratio_diffs1);

      // find the median value for each objective from cloud and compare2
      centroid_diffs_median=utl.getMedian(centroid_diffs2);
      volume_diffs_median=utl.getMedian(volume_diffs2);
      aspect_ratio_diffs_median=utl.getMedian(aspect_ratio_diffs2);

      // normalize diffs by dividing by median difference for each objective from cloud and compare 2
      centroid_diffs2_norm=centroid_diffs2/centroid_diffs_median; // use vectorized row operations from library Eigen
      volume_diffs2_norm=volume_diffs2/volume_diffs_median;       
      aspect_ratio_diffs2_norm=aspect_ratio_diffs2/aspect_ratio_diffs_median;

      // normalize diffs by dividing by median difference for each objective from cloud and compare 1
      //centroid_diffs2_norm=centroid_diffs2/getMedian(centroid_diffs2); // use vectorized row operations from library Eigen
      //volume_diffs2_norm=volume_diffs2/getMedian(volume_diffs2);      
      //aspect_ratio_diffs2_norm=aspect_ratio_diffs2/getMedian(aspect_ratio_diffs2);

      //scores=centroid_diffs_norm+volume_diffs_norm+aspect_ratio_diffs_norm;
      // return the score as the sum of the normalized terms for each pair  

      std::cout<<"DEBUG: finished normalizing diffs"<<std::endl;

      return  centroid_diffs1_norm+volume_diffs1_norm+aspect_ratio_diffs1_norm+
              centroid_diffs2_norm+volume_diffs2_norm+aspect_ratio_diffs2_norm; 
      
    }


    // function to find best match between single pointcloud and two sets of clusters using scoreClustersMulti2() 
    PointCloudPtr matchCloudToClustersMulti2(PointCloud &cloud, PointCloudVec compares1, PointCloudVec compares2, int verbosity){

      Eigen::VectorXd scores;
      scores=scoreCloudsMulti2(cloud, compares1, compares2);  

      // find pair with min sum objective difference using median normalized differences, replace with std::min for speed
      int j_min;
      double score_min;

      j_min=0; // default value for the search index, in case it is not set
      score_min=scores[0]; // assume first may be the minimum

      for(int j=0; j<compares1.size(); j++){ // re-check each possible pair
        if (scores[j]<score_min){ // find the lowest score
          score_min=scores[j];
          j_min=j;            // record the index of the lowest score

        }
      }          

      // printing for debugging, optional, controlled by fn arg
      if(verbosity>1){ // show the diff values from the search each iteration

      }else if(verbosity>0){ // show the results of the search after complete
        std::cout <<"cloud has "<< cloud.size()<< " points " 
                  <<" and match from compares1 has "<<compares1[j_min]->size()<< " points"<<std::endl;           
      }
      
      return compares1[j_min];  // return the compare1 with the best score to matches, this may need changing
    }

      
    // function to find best 1 to 1 correlation between two sets of clusters
    // assumes size of clusters is less than or equal to size of compares to ensure 1-1 correspondence
    // this version checks all n^2 matches before removing any from the compare set, this is O(n^3), so it may be slow! 
    PointCloudVec matchClusters3(PointCloudVec clusters, PointCloudVec compares, int verbosity){

      double score, score_min;
      std::vector<double> scores(clusters.size()); // vector of scores, for debugging purposes

      PointCloudVec matches;
      matches=clusters; // copying to get size, just testing, there is probably a better way... 

      // vectors to contain the search set indices, these are the indexes to look in clusters and compares
      std::vector<int> cluster_indices(clusters.size()), compare_indices(compares.size()); 
      std::iota (std::begin(cluster_indices), std::end(cluster_indices), 0); // fill vector with consecutive integers
      std::iota (std::begin(compare_indices), std::end(compare_indices), 0); // fill vector with consecutive integers
      // vector to contain match original index from compare vector, for debugging only
      std::vector<int> original_indices(clusters.size());

      std::vector<int>::iterator it, jt, it_min, jt_min;

      if (clusters.size()<=compares.size()){  // clusters1 has fewer clusters than clusters2  (input error checking)
         
        for (int h=0; h<clusters.size(); h++){ // loop across each cluster in clusters, to find a best match for each
          // seed the search with the score of first pair before the outside loop
          score_min=scoreClouds(*clusters[cluster_indices[0]], *compares[compare_indices[0]], verbosity);  
          // for each cluster in clusters1 find best match from clusters2 
          it_min=cluster_indices.begin(); // default value for it_min in case it is not assigned in search
          for (it=cluster_indices.begin(); it != cluster_indices.end(); it++){ 
            
            jt_min=compare_indices.begin(); // default value for jt_min in case it is not assigned in search
            for (jt=compare_indices.begin(); jt != compare_indices.end(); jt++){
         
              score=scoreClouds(*clusters[*it], *compares[*jt], verbosity); // get the score of the ith cluster and jth compare
              if(verbosity>1){
                std::cout<<"clusters["<<*it<<"] (size:" <<clusters[*it]->size()<<") and compares["<<*jt
                         <<"] (size:"<<compares[*jt]->size()<<") have a score "<<score<<std::endl<<std::endl;
              } 

              if (score<score_min){ // check if this is the best score yet
                score_min=score;    // save the min score
                it_min=it;     // save the iterator to the min score cluster
                jt_min=jt;     // save the iterator to the min score compare

              }
            }
          }

          // after checking all potential matches, 
          // assign the best match into the vector of matches with the recorded index for the correct cluster
          matches.at(*it_min)=compares[*jt_min]; // you could just index it, but at() is the cpp way 
          scores.at(*it_min)=score_min;          // save the score, for debugging the cost function
          original_indices.at(*it_min)=*jt_min;  // save the original index of the compare cluster, for debugging only  
          if(verbosity>1){
            std::cout<<"on iteration "<<h<<" the best match was found between clusters["<<*it_min
                   << "] and compares["<<*jt_min<<"] with score "<<score_min<<std::endl;
            std::cout<<"clusters["<<*it_min<<"] and compares["<<*jt_min<<"] were removed from the search sets"<<std::endl;
          }
          cluster_indices.erase(it_min);  // remove the cluster index from the set of clusters indices
          compare_indices.erase(jt_min);  // remove the match index from the set of compares for 1-1 correspondence, do this last      
        }
        
        if (verbosity>0){  
          for (int k=0; k<clusters.size(); k++){
            std::cout<<"clusters["<<k<<"] (size:" <<clusters[k]->size()<<") matched with compares["<<original_indices[k]
                     <<"](size:"<<compares[original_indices[k]]->size()<<") which is now matches["<<k<<"] (size:"
                     <<matches[k]->size()<<")"<<std::endl;
            std::cout<<"the comparison score between clusters["<<k<<"] and compares["<<original_indices[k]<<"] is "<<scores[k]
            <<std::endl;
          }
        }
      
      }else{       // compares has fewer clusters than clusters, empty return 
        std::cout<<"warning: ( clusters.size() <= compares.size() ) failed, no matches returned"<<std::endl;
      }

      //std::cout<<"matches contains "<<matches.size()<<" clusters after matching complete"<<std::endl;
      return matches;
    } // end of matchClusters3() function
    


    // function to convert to eigen vector 3 double
    Eigen::Vector3d toEigenVector3d(std::vector<double> input){

      Eigen::Vector3d output;
      output[0]=input[0]; 
      output[1]=input[1]; 
      output[2]=input[2];
       
      return output;
    }
  

    // PUBLIC attributes

    // other parameters from the config file (these do not need to public)
    bool auto_bounds=0;
    bool save_output, translate_output, automatic_bounds, use_clustering, new_scan, transform_input;
    std::string package_path, training_path, test_path, output_path, training_file, test_file, output_file,
                training_view1_file, training_view2_file, training_view3_file, training_view4_file, 
                training_merged_file, training_inliers_file,  
                test_view1_file, test_view2_file, test_view3_file, test_view4_file, 
                test_merged_file, test_inliers_file; 

    std::vector<double> bounding_box, pre_rotation, pre_translation;
    double voxel_size;
    
    // parameters for the Euclidean Cluster Extraction, values defined in config file
    float euclidean_thresh;  
    int euclidean_min_size, euclidean_max_size, euclidean_max_clusters; 
    
    // parameters for the Color-Based Region-Growing Segmentation, values defined in config file
    float color_distance_thresh, color_point_thresh, color_region_thresh;
    int color_min_size, color_max_clusters;  
   
    // topic for generic cloud publisher
    std::string cloud_topic;


  private:

    // PRIVATE attributes

    // ros objects
    ros::NodeHandle node;
    ros::Rate rate;       // rate might be useful in 'public', but this is the proper place

    // these top three  may not be used currently, check on this team
    // std::vector<ros::Publisher> pub_color, pub_euclidean, pub_intersection;
    // generic publishers, 
    std::vector<ros::Publisher> pub_clouds;
    std::vector<ros::Publisher> pub_clusters;
    int pub_idx;


};


int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"seam_detection");
 
  // instantiate object sd from the SeamDetection class, defined in this file for now
  SeamDetection sd;
  
  // load ROS parameters, modify values in seam-detection.yaml
  sd.loadConfig(); 

  // instantiate object utl from the CloudUtils class, see include/cloudutils.h 
  CloudUtils util;
 
  // step 0 - reconstruction reconstruction by merging different views
  // step 0.1 - training image
  
  // step0 - image reconstruction by merging different views

  PointCloud::Ptr cloud_view1 (new PointCloud);
  PointCloud::Ptr cloud_view2 (new PointCloud);
  PointCloud::Ptr cloud_view3 (new PointCloud);
  PointCloud::Ptr cloud_view4 (new PointCloud); 
  PointCloud::Ptr cloud_merged (new PointCloud);
 
  util.loadCloud(*cloud_view1, sd.training_view1_file);
  util.loadCloud(*cloud_view2, sd.training_view2_file);
  util.loadCloud(*cloud_view3, sd.training_view3_file);
  util.loadCloud(*cloud_view4, sd.training_view4_file);

  PointCloudVec cloud_views;
  cloud_views.push_back(cloud_view1);
  cloud_views.push_back(cloud_view2);
  cloud_views.push_back(cloud_view3);
  cloud_views.push_back(cloud_view4);

  cloud_merged=util.mergeClusters(cloud_views);
  util.publishCloud(*cloud_view1, "training_cloud_view1", "base_link");
  util.publishCloud(*cloud_view2, "training_cloud_view2", "base_link");
  util.publishCloud(*cloud_view3, "training_cloud_view3", "base_link");
  util.publishCloud(*cloud_view4, "training_cloud_view4", "base_link");
  util.publishCloud(*cloud_merged, "training_cloud_merged", "base_link");

  // save resulting merge to pcd file
  util.saveCloud(*cloud_merged, sd.training_merged_file);
  
  // step 0.2 - test image reconstruction by merging different views
  //PointCloud::Ptr cloud_view1 (new PointCloud);
  //PointCloud::Ptr cloud_view2 (new PointCloud);
  //PointCloud::Ptr cloud_view3 (new PointCloud);
  //PointCloud::Ptr cloud_view4 (new PointCloud); 
  PointCloud::Ptr test_cloud_merged (new PointCloud);
 
  util.loadCloud(*cloud_view1, sd.test_view1_file);
  util.loadCloud(*cloud_view2, sd.test_view2_file);
  util.loadCloud(*cloud_view3, sd.test_view3_file);
  util.loadCloud(*cloud_view4, sd.test_view4_file);

  PointCloudVec test_cloud_views;
  test_cloud_views.push_back(cloud_view1);
  test_cloud_views.push_back(cloud_view2);
  test_cloud_views.push_back(cloud_view3);
  test_cloud_views.push_back(cloud_view4);

  test_cloud_merged=util.mergeClusters(test_cloud_views);
  util.publishCloud(*cloud_view1, "test_cloud_view1", "base_link");
  util.publishCloud(*cloud_view2, "test_cloud_view2", "base_link");
  util.publishCloud(*cloud_view3, "test_cloud_view3", "base_link");
  util.publishCloud(*cloud_view4, "test_cloud_view4", "base_link");
  util.publishCloud(*test_cloud_merged, "test_cloud_merged", "base_link");

  // save resulting merge to pcd file
  util.saveCloud(*test_cloud_merged, sd.test_merged_file);
  
  std::cout<<"|----------- Step 0 Complete ----------|"<<std::endl;  
  
 
  // [Steps 1-3] - use 'training' image of target object on clean table

  // Step 1 - load the 'training' pointcloud from pcd file 

  PointCloud::Ptr training_input (new PointCloud);
  PointCloud::Ptr training_downsampled (new PointCloud);
  PointCloud::Ptr training_transformed (new PointCloud);
  PointCloud::Ptr training_bounded (new PointCloud);
  PointCloud::Ptr training_inliers (new PointCloud);
  //PointCloudNormal::Ptr training_smoothed (new PointCloudNormal); 
  
  util.loadCloud( *training_input, sd.training_file );
  
  // Step 1.5 - perform voxel-downsampling, pre-transformation, and bounding-box on the training cloud
  CloudFilter filter;   // class defined in seam_detection, see include/cloudfilter.h
  filter.loadConfig("filter_dataset");
   
  filter.downsampleCloud(*training_input, *training_downsampled, sd.voxel_size); 
  filter.transformCloud(*training_downsampled, *training_transformed, sd.pre_rotation, sd.pre_translation);
  filter.boundCloud(*training_transformed, *training_bounded, sd.bounding_box);
  filter.removeOutliers(*training_bounded, *training_inliers); 
  //filter.smoothCloud(*training_bounded, *training_smoothed);

  // save resulting filtered image to pcd file
  util.saveCloud(*training_inliers, sd.training_inliers_file);
  
  std::cout<<"training_smoothed has "<<training_bounded->size()<<" points"<<std::endl;
  
  // show the input training clouds in rviz
  util.publishCloud(*training_input, "/training_input", "base_link"); 
  util.publishCloud(*training_downsampled, "/training_downsampled", "base_link");
  util.publishCloud(*training_transformed, "/training_transformed", "base_link"); 
  util.publishCloud(*training_bounded, "/training_bounded", "base_link");
  util.publishCloud(*training_inliers, "/training_inliers", "base_link");
  //util.publishCloud(*training_smoothed, "/training_smoothed", "base_link");

  std::cout<<"|----------- Step 1 Complete ----------|"<<std::endl;    
  
 
  // Step 2 - extract clusters from training cloud using euclidean and color algorithms
  PointCloudVec training_euclidean_clusters, training_color_clusters;

  // perform Euclidean cluster extraction
  training_euclidean_clusters=sd.extractEuclideanClusters(*training_inliers); 
  // perform Color Based Region Growing cluster extraction
  training_color_clusters=sd.extractColorClusters(*training_inliers);
    

  std::cout<<"training_euclidean_clusters size:"<<training_euclidean_clusters.size()<<std::endl;
  std::cout<<"training_color_clusters size:"<<training_color_clusters.size()<<std::endl;

  // show the extracted 'training' clusters in rviz 
  // show the euclidean and color based clusters for the training cloud
  util.publishClusters(training_euclidean_clusters, "/training_euclidean");
  util.publishClusters(training_color_clusters, "/training_color");     

  // smooth the bounded training cloud and repeat the color clustering
  //PointCloudNormalVec training_smoothed_color_clusters;
  //std::vector<typename pcl::PointCloud<PointNT>::Ptr, 
  //            Eigen::aligned_allocator<typename pcl::PointCloud<PointNT>::Ptr> > 
  // training_smoothed_color_clusters;

  //using PointCloudPtrType = typename pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr;
  //using AllocatorType = Eigen::aligned_allocator<PointCloudPtrType>;
  //using VectorType = std::vector<PointCloudPtrType, AllocatorType>;
  //VectorType training_smoothed_color_clusters;

  //training_smoothed_color_clusters=sd.extractColorClustersT(*training_smoothed);
  
  std::cout<<"|----------- Step 2 Complete ----------|"<<std::endl;  
  
  
  //sd.publishClustersT(training_smoothed_color_clusters, "/training_smoothed_color"); 

  // Step 3 - choose proper euclidean clusters and color clusters using correlation routine 
  // between training euclidean and training color 
  // controls debug printing, 0-no print, 1-print search results, 2-print search data and search results 
  int debug_level=1;
  PointCloudVec training_matches; // this vector contains pointers to the original clusters data
  float centroid_wt=1;
  training_matches=sd.matchClustersMulti(training_euclidean_clusters, training_color_clusters, debug_level, centroid_wt); 
  
  // show the matches to the clusters in rviz
  util.publishClusters(training_matches, "/training_match");
  
 
  // 3.5 - find intersection of the training data (training_euclidan_clusters[0] , training_matches[0])
  // memory allocation because the intersection cloud data will be copied to a new pointcloud
  PointCloudPtr training_intersection (new PointCloud); 
  PointCloudPtr training_union (new PointCloud); 
  
  int idx=0;
  util.getCloudIntersection(*training_euclidean_clusters[idx], *training_matches[idx], *training_intersection);
  std::cout<<"training_intersection has "<<training_intersection->size()<<" points"<<std::endl;    

  util.getCloudUnion(*training_euclidean_clusters[idx], *training_matches[idx], *training_union);
  std::cout<<"training_union has "<<training_union->size()<<" points"<<std::endl;
    
  util.publishCloud(*training_intersection, "/training_intersection", "base_link"); // show in rviz
  util.publishCloud(*training_union, "/training_union", "base_link"); // show in rviz

  std::cout<<"|----------- Step 3 Complete ----------|"<<std::endl;  
   
  // [Steps 4-7] - use 'test' image of target object on cluttered table
  PointCloud::Ptr test_input (new PointCloud);
  PointCloud::Ptr test_downsampled (new PointCloud);
  PointCloud::Ptr test_transformed (new PointCloud);
  PointCloud::Ptr test_bounded (new PointCloud);
  PointCloud::Ptr test_inliers (new PointCloud); 
  //PointCloudNormal::Ptr test_smoothed (new PointCloudNormal);
  
    
  // Step 4 - load the 'test' pointcloud from pcd file (this is the cluttered table)
  util.loadCloud(*test_input, sd.test_file);
  
  std::cout<<"|----------- Step 4 Complete ----------|"<<std::endl;  

  
  // Step 5 - voxel-downsampling, pre-transformation, and bounding-box 
  // on the test cloud (same params used as in step 1.5)
  filter.downsampleCloud(*test_input, *test_downsampled, sd.voxel_size); 
  filter.transformCloud(*test_downsampled, *test_transformed, sd.pre_rotation, sd.pre_translation);
  filter.boundCloud(*test_transformed, *test_bounded, sd.bounding_box); 
  filter.removeOutliers(*test_bounded, *test_inliers); 

  // show the input test clouds in rviz
  util.publishCloud(*test_input, "/test_input", "base_link"); 
  util.publishCloud(*test_downsampled, "/test_downsampled", "base_link");
  util.publishCloud(*test_transformed, "/test_transformed", "base_link"); 
  util.publishCloud(*test_bounded, "/test_bounded", "base_link");
  util.publishCloud(*test_inliers, "/test_inliers", "base_link");

  // save resulting filtered test images to pcd file
  util.saveCloud(*test_inliers, sd.test_inliers_file);

  std::cout<<"|----------- Step 5 Complete ----------|"<<std::endl;  
 
 
  // Step 6 - extract clusters from test cloud using euclidean and color algorithms
  PointCloudVec test_euclidean_clusters, test_color_clusters;
 
  // preform Euclidean cluster extraction
  test_euclidean_clusters=sd.extractEuclideanClusters(*test_inliers); 
  // preform Color Based Region Growing cluster extraction
  test_color_clusters=sd.extractColorClusters(*test_inliers);
 
  std::cout<<"test_euclidean_clusters size:"<<test_euclidean_clusters.size()<<std::endl;
  std::cout<<"test_color_clusters size:"<<test_color_clusters.size()<<std::endl;
  util.publishClusters(test_euclidean_clusters, "/test_euclidean"); // show the euclidean and color based clusters  
  util.publishClusters(test_color_clusters, "/test_color");         // for the test cloud  

  std::cout<<"|----------- Step 6 Complete ----------|"<<std::endl;  
   
  // Step 7 - correlate test euclidean clusters to test color clusters, use multi objective function 
  // this should be wrapped up in a function to clean things up
  PointCloudVec test_matches;
  centroid_wt=2;
  test_matches=sd.matchClustersMulti(test_euclidean_clusters, test_color_clusters, debug_level, centroid_wt); 
  // show the matched clusters in rviz
  util.publishClusters(test_matches, "/test_match");
  
  std::cout<<"|----------- Step 7.1 Complete ----------|"<<std::endl;  

   
  // Step 7.5 - Extract intersection of the test data (ALL test_euclidan_clusters[:] , all test_matches[:]) 
  PointCloudVec test_intersections; // vector of pointcloud points, dynamic sized  
  PointCloudVec test_unions; // vector of pointcloud points, dynamic sized 

  int min_size=1; // min points in an intersection
  test_intersections=util.getClusterIntersections(test_euclidean_clusters, test_matches, min_size);
  test_unions=util.getClusterUnions(test_euclidean_clusters, test_matches, min_size);

  std::cout<<"test_intersections has "<<test_intersections.size()<<" clouds"<<std::endl;
  util.publishClusters(test_intersections, "/test_intersection");
   
  std::cout<<"test_unions has "<<test_unions.size()<<" clouds"<<std::endl;
  util.publishClusters(test_unions, "/test_unions");
  
  std::cout<<"|----------- Step 7 Complete ----------|"<<std::endl;  
    
  
  // Step 8 compare 'training' target from steps 1-3 to correlated 'test_intersection' clusters from steps 4-7 
  PointCloud::Ptr final_intersection, final_union;
  centroid_wt=0;
  final_intersection=sd.matchCloudToClustersMulti(*training_intersection, test_intersections, debug_level, centroid_wt);  
  final_union=sd.matchCloudToClustersMulti(*training_union, test_unions, debug_level, centroid_wt); 
  
  // show the matching target intersection and union
  std::cout<<"final_intersection has "<<final_intersection->size()<<" points"<<std::endl;
  util.publishCloud(*final_intersection, "/final_intersection", "base_link");        
  
  std::cout<<"final_union has "<<final_union->size()<<" points"<<std::endl;
  util.publishCloud(*final_union, "/final_union", "base_link"); 
 
 
  std::cout<<"|----------- Step 8 Complete ----------|"<<std::endl;  
    
  // save resulting filtered image to pcd file
  util.saveCloud(*final_union, sd.output_file); 

  std::cout<<"|----------- seam_detection complete ----------|"<<std::endl;  
  ros::spin();

  return 0;
}

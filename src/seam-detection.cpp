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
    SeamDetection(): rate(5), pub_idx(0) { // ROS::Rate rate(5) is in intializer list
      
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"|---------- SeamDetection v1.9 ----------|"<<std::endl;
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

      // allocate memory for pointclouds member attributes
      test_input = new PointCloud;
      test_downsampled = new PointCloud;
      test_transformed = new PointCloud;
      test_bounded = new PointCloud;  
      //test_target = new PointCloud;

      training_input = new PointCloud;
      training_downsampled = new PointCloud;
      training_transformed = new PointCloud;
      training_bounded = new PointCloud;
      //training_target = new PointCloud;
      
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
      node.getParam("seam_detection/training_file", training_file);
      node.getParam("seam_detection/test_file", test_file);    
      node.getParam("seam_detection/output_file", output_file);

      // generate absolute file paths to inputs (does this belong here?)
      training_path=package_path+'/'+training_file;
      test_path=package_path+'/'+test_file;
      output_path=package_path+'/'+output_file;
      
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
      node.getParam("seam_detection/bounding_box",  bounding_box_vec);
      for(unsigned i=0; i < bounding_box_vec.size(); i++){
        bounding_box[i]=bounding_box_vec[i]; // copy from vector to array 
      }

      std::vector<double> pre_rotation_vec, pre_translation_vec;
      node.getParam("seam_detection/pre_rotation",  pre_rotation_vec);
      node.getParam("seam_detection/pre_translation",  pre_translation_vec);
      for(unsigned i=0; i < pre_rotation_vec.size(); i++){
        pre_rotation[i]=pre_rotation_vec[i]; // copy from std vector to eigen vector3f 
        pre_translation[i]=pre_translation_vec[i]; 
      }

      
    
      return 0;
    }
    

    // function to load pointcloud from PCD file as defined in config
    // hardcoded for training and test class members, consider generalizing ...
    int loadClouds(std::string training_file, std::string test_file){

      std::cout<<"|---------- SeamDetection::LoadCloud - loading PCD files ----------|"<<std::endl;

      std::cout << "Loading training input file:" << training_path << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (training_path, *training_input) == -1)
      {
          std::cout<<"Couldn't read training input file:"<<training_path;
          return (-1);
      }
      std::cout << "Loaded training input file: "<< training_path <<std::endl<<
        training_input->width * training_input->height << " data points from "<< training_path << std::endl;

      std::cout << "Loading test input file:" << test_path << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (test_path, *test_input) == -1)
      {
          std::cout<<"Couldn't read source input file:"<<test_path;
          return (-1);
      }
      std::cout << "Loaded test input file: "<< test_path <<std::endl<<
        test_input->width * test_input->height << " data points from "<< test_path << std::endl;
 
      return 0;  
    } 

    // function to load pointcloud from PCD file as defined in config
    int loadCloud(std::string file_name, PointCloud &input){

      std::cout<<"|---------- SeamDetection::LoadCloud - loading PCD file ----------|"<<std::endl;

      std::string file_path;
      file_path=package_path+"/"+file_name;

      std::cout << "Loading input pointcloud file: " << file_path << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_path, input) == -1)
      {
          std::cout<<"Failed to load input pointcloud file: "<< training_path <<std::endl;
          return (-1);
      }
      std::cout << "Loaded "<<input.width * input.height << " data points from input pointcloud file: "<< file_path <<std::endl;
      return 0;  
    } 


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
        std::cout<<"leaf_size>0 failed, no voxel filtering"<< std::endl;
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


    // function to perform Euclidean Cluster Extraction  
    PointCloudVec extractEuclideanClusters(PointCloud &input,  int min_size, int max_size, double tolerance){

      PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the training cloud
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
      PointCloudVec clusters;

      int j = 0;
      for (const auto& cluster : cluster_indices) 
      {
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

      for (int i = 0; i < clusters.size(); i++){
        std::cout << "euclidean cluster " << i << " has " << clusters[i]->size() << " points " << std::endl;
      }

      std::cout<< "euclidean_clusters size: "<< clusters.size() <<std::endl;

      //pcl::PointCloud <PointT>::Ptr colored = ec.getColoredCloud (); // getColoredCloud not available
      //pcl::copyPointCloud(*colored, *cloud_color); // copy to member attribute
        
      return clusters;
    }


    // function to compare cloud size, primarily for std::sort()
    static bool CompareSize(PointCloudPtr cloud_a, PointCloudPtr cloud_b){

      return cloud_a->size()>cloud_b->size(); // for sorting by greatest to least number of points in cloud

    }
  

    // function to perform Color Based Region Growing Cluster Extraction 
    PointCloudVec extractColorClusters(PointCloud &input, int min_size, double dist_thresh, double reg_color_thresh, double pt_color_thresh){
      
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
      //pcl::copyPointCloud(*colored, *recolored_cloud); // copy to member attribute
      // recolored cloud not used in workflow

      // instantiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
      PointCloudVec clusters;

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

      for (int i = 0; i < clusters.size(); i++){
        std::cout << "color cluster " << i << " has " << clusters[i]->size() << " points " << std::endl;
      }

      std::cout<< "color_clusters size: "<< clusters.size() <<std::endl;

      // sort the cluster using user-defined compare function defined above 
      std::sort(clusters.begin(), clusters.end(), CompareSize);

      for (int i = 0; i < clusters.size(); i++){
        std::cout << "sorted color cluster " << i << " has " << clusters[i]->size() << " points " << std::endl;
      }  

      return clusters;

    }


    // function to find the minimum oriented bounding box of a cloud using principle component analysis
    // void getPCABox(PointCloud &input, Eigen::Quaternionf& bbox_quaternion, Eigen::Vector3f& bbox_translation, Eigen::Vector3f& bbox_dimensions){

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

    void getPCABox(PointCloud &input, Eigen::Quaternionf& rotation, Eigen::Vector3f& translation, Eigen::Vector3f& dimension, Eigen::Matrix3f eigen_vectors){

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

      std::vector < Eigen::Quaternionf > quaternions; // vector of quaternions, maybe not the best solution... send me a better one, 2D array containing quats? eh...
      std::vector < Eigen::Vector3f > translations;   // these could be in a 2D array, but then the syntax would not match
      std::vector < Eigen::Vector3f > dimensions;

      for (int i = 0; i < clouds.size(); i++){
        
        // get the pose and size of the minimum bounding box for each cluster
        Eigen::Quaternionf quaternion; // this is a temp variable to get the eigen::quaternion from the function which will be added to quaternions vector
        Eigen::Vector3f translation, dimension; // these are also temp vars for the same purpose, there is probably a better way to do this ... 
        getPCABox(*clouds[i], quaternion, translation, dimension);  // does not work (compiles but throws runtime error), pick up here! 
        
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
    double scoreClouds(PointCloud &cloud1, PointCloud &cloud2, int verbosity){

      double score=100, f1, f2, f3, f4, 
             distance_x, distance_y, distance_z,
             volume1, volume2, 
             aspect_ratio1, aspect_ratio2,
             difference_x, difference_y, difference_z;

      // find the pca min bounding box for the first cloud
      Eigen::Quaternionf quaternion1; 
      Eigen::Vector3f translation1, dimension1;
      Eigen::Matrix3f eigenvectors1;
      getPCABox(cloud1, quaternion1, translation1, dimension1, eigenvectors1);

      // find the pca min bounding box for the second cloud 
      Eigen::Quaternionf quaternion2; 
      Eigen::Vector3f translation2, dimension2;
      Eigen::Matrix3f eigenvectors2;  
      getPCABox(cloud2, quaternion2, translation2, dimension2, eigenvectors2);

      // calculate separate terms for the objective function between the two clouds

      // term1 - position of centroid
      distance_x=translation1[0]-translation2[0];
      distance_y=translation1[1]-translation2[1];
      distance_z=translation1[2]-translation2[2];
      f1 = pow(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2), 1.0/2.0); // square root of sum of squared component distances between centroids - l
      //f1 = 0; // disabled temporarily   
      //std::cout<<"f1: "<<f1<<std::endl;

      // term2 - volume of bounding box
      volume1=dimension1[0]*dimension1[1]*dimension1[2];
      volume2=dimension2[0]*dimension2[1]*dimension2[2];
      f2 = pow(std::abs(volume1-volume2),1.0/3.0); // cube root of difference in volume - l
      f2 = std::abs(volume2-volume2);               
      //std::cout<<"f2: "<<f2<<std::endl;

      // term3 - aspect ratio of bounding box 
      aspect_ratio1=  dimension1.maxCoeff()/dimension1.minCoeff(); 
      aspect_ratio2=  dimension2.maxCoeff()/dimension2.minCoeff(); 
      f3 = pow(pow(aspect_ratio1 - aspect_ratio2, 2), 1.0/2.0); // square root of squared difference in aspect ratios - l
      //f3 = 0; // disabled temporarily
      //std::cout<<"f3: "<<f3<<std::endl;

      // term4 - orientation of bounding box
      difference_x=dimension1[0]-dimension2[0]; // this does not seem right, does not contain orientation info...
      difference_y=dimension1[1]-dimension2[1]; // need to use projection onto fixed framed
      difference_z=dimension1[2]-dimension2[2]; 

      f4 = pow(pow(difference_x,2)+pow(difference_y,2)+pow(difference_z,2), 1.0/2.0); // square root of sum of square dimension differences - l
      f4 = 0; // disabled temporarily 
      //std::cout<<"f4: "<<f4<<std::endl;
      
      // objective function value is sum of terms 
      score=f1+f2+f3+f4;
      
      if(verbosity>1){
        std::cout<<"translation1: "<<std::endl<<"["<<translation1[0]<<","<<translation1[1]<<","<<translation1[2]<<"]"<<std::endl;
        std::cout<<"translation2: "<<std::endl<<"["<<translation2[0]<<","<<translation2[1]<<","<<translation2[2]<<"]"<<std::endl;

        std::cout<<"dimension1: "<<std::endl<<"["<<dimension1[0]<<","<<dimension1[1]<<","<<dimension1[2]<<"]"<<std::endl;
        std::cout<<"volume1: "<<volume1<<std::endl;
        std::cout<<"dimension2: "<<std::endl<<"["<<dimension2[0]<<","<<dimension2[1]<<","<<dimension2[2]<<"]"<<std::endl;
        std::cout<<"volume2: "<<volume2<<std::endl;

        std::cout<<"eigenvectors1: "<<std::endl<<"[" << eigenvectors1(0,0)<<","<< eigenvectors1(0,1)<<","<< eigenvectors1(0,2)<<std::endl
                                      << eigenvectors1(1,0)<<","<< eigenvectors1(1,1)<<","<< eigenvectors1(1,2)<<std::endl
                                      << eigenvectors1(2,0)<<","<< eigenvectors1(2,1)<<","<< eigenvectors1(2,2)<<"]"<<std::endl;
        std::cout<<"eigenvectors2: "<<std::endl<<"[" << eigenvectors2(0,0)<<","<< eigenvectors2(0,1)<<","<< eigenvectors2(0,2)<<std::endl
                                      << eigenvectors2(1,0)<<","<< eigenvectors2(1,1)<<","<< eigenvectors2(1,2)<<std::endl
                                      << eigenvectors2(2,0)<<","<< eigenvectors2(2,1)<<","<< eigenvectors2(2,2)<<"]"<<std::endl;
        std::cout<<"objective function value: "<<score<<std::endl;                              
      }
      
      return score=f1+f2+f3+f4;

    }

    // function to find best 1 to 1 correlation between two sets of clusters
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


    // function to find best match between sets of clusters using multi-objective optimization
    PointCloudVec matchClustersMulti(PointCloudVec clusters, PointCloudVec compares, int verbosity){

      PointCloudVec matches;
      matches=clusters; // make a copy just to get the size, change to an empty copy later

      std::vector<double> scores(compares.size()); // vector of scores, for debugging purposes
      std::vector<double> centroid_diffs(compares.size()), // vectors of differences 
                          volume_diffs(compares.size()), 
                          aspect_ratio_diffs(compares.size());

      std::vector<double> centroid_diffs_norm(compares.size()), // normalized vectors of differences 
                          volume_diffs_norm(compares.size()), 
                          aspect_ratio_diffs_norm(compares.size());                    

      double distance_x, distance_y, distance_z,
      volume1, volume2, 
      aspect_ratio1, aspect_ratio2,
      difference_x, difference_y, difference_z;

      double centroid_diffs_median, volume_diffs_median, aspect_ratio_diffs_median;
      int n;
      if (clusters.size()<=compares.size()){  // clusters has fewer clusters than compares
        n=clusters.size();
      }else{
        n=compares.size();
        std::cout<<"warning: ( clusters.size() <= compares.size() ) failed, matching clusters[1:compares.size()] to compares[:]"<<std::endl; 
      } 
         
      for (int i=0; i<n; i++){  // compare each cluster in clusters to each cluster in compares 
                                      
        for (int j=0; j<compares.size(); j++){

          // find the pca min bounding box for the ith cloud in clusters
          Eigen::Quaternionf quaternion1; 
          Eigen::Vector3f translation1, dimension1;
          Eigen::Matrix3f eigenvectors1;
          getPCABox(*clusters[i], quaternion1, translation1, dimension1, eigenvectors1);

          // find the pca min bounding box for the jth cloud in compares
          Eigen::Quaternionf quaternion2; 
          Eigen::Vector3f translation2, dimension2;
          Eigen::Matrix3f eigenvectors2;
          getPCABox(*compares[j], quaternion2, translation2, dimension2, eigenvectors2);

          // calculate the objective differences for each pair 
          // term1 - position of centroid
          distance_x=translation1[0]-translation2[0];
          distance_y=translation1[1]-translation2[1];
          distance_z=translation1[2]-translation2[2];
          centroid_diffs.at(j) = pow(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2), 1.0/2.0); // square root of sum of squared component distances between centroids - l
          // term2 - volume of bounding box
          volume1=dimension1[0]*dimension1[1]*dimension1[2];
          volume2=dimension2[0]*dimension2[1]*dimension2[2];
          volume_diffs.at(j) = std::abs(volume1-volume2);
          // term3 - aspect ratio of bounding box 
          aspect_ratio1=  dimension1.maxCoeff()/dimension1.minCoeff(); 
          aspect_ratio2=  dimension2.maxCoeff()/dimension2.minCoeff(); 
          aspect_ratio_diffs.at(j)= std::abs(aspect_ratio1 - aspect_ratio2); // square root of squared difference in aspect ratios - l

        }

        // find the median value for each objective 
        centroid_diffs_median=getMedian(centroid_diffs);
        volume_diffs_median=getMedian(volume_diffs);
        aspect_ratio_diffs_median=getMedian(aspect_ratio_diffs);
        
        // find pair with min sum objective difference using median normalized differences 
        double score, score_min;
        int j_min=0; // default value for the search index, in case it is not set

        // seed the minimization with the first set of differences 
        score_min=centroid_diffs_norm[j_min]+volume_diffs_norm[j_min]+aspect_ratio_diffs_norm[j_min];

        for(int j=0; j<compares.size(); j++){ // re-check each possible pair
          // normalize diffs by dividing by median difference for each objective 
          centroid_diffs_norm[j]=centroid_diffs[j]/centroid_diffs_median;
          volume_diffs_norm[j]=volume_diffs[j]/volume_diffs_median;
          aspect_ratio_diffs_norm[j]=aspect_ratio_diffs[j]/aspect_ratio_diffs_median;  

          // calculate the score as the sum of the normalized diffs 
          score=centroid_diffs_norm[j]+volume_diffs_norm[j]+aspect_ratio_diffs_norm[j];

          if (score<score_min){ // find the lowest score
            score_min=score;
            j_min=j;            // record the index of the lowest score

          }
        }          

        // add the compare with the best score to matches
        matches.at(i)=compares[j_min];
        // remove the match from the compare set for next iteration
        compares.erase(compares.begin()+j_min);

        if(verbosity>1){ // show the values as the search is performed

          std::cout<<std::endl<<"iteration "<<i<<std::endl;

          for (int j=0; j<scores.size(); j++){
            std::cout <<"centroid_diffs["<<j<<"]: "<<centroid_diffs[j]
                      <<", volume_diffs["<<j<<"]: "<<volume_diffs[j]
                      <<", aspect_ratio_diffs["<<j<<"]: "<<aspect_ratio_diffs[j]<<std::endl;
          }
          std::cout <<"centroid_diffs_median: "<<centroid_diffs_median
                    <<", volume_diffs_median:"<<volume_diffs_median
                    <<", aspect_ratio_diffs_median: "<<aspect_ratio_diffs_median<<std::endl; 
          std::cout<<"iteration "<<i<<" normalized with median" <<std::endl;
          for (int j=0; j<scores.size(); j++){
            std::cout <<"centroid_diffs_norm["<<j<<"]: "<<centroid_diffs_norm[j]
                      <<", volume_diffs_norm["<<j<<"]: "<<volume_diffs_norm[j]
                      <<", aspect_ratio_diffs_norm["<<j<<"]: "<<aspect_ratio_diffs_norm[j]<<std::endl;
          }
        }
      }  
      
      if(verbosity>0){ // show the results of the search after complete
        for (int k=0; k<clusters.size(); k++){
          std::cout <<"cluster["<<k<<"] has "<< clusters[k]->size()<< " points " 
                    <<" and matches["<<k<<"] has "<<matches[k]->size()<< " points"<<std::endl;
        }             
      }
      return matches;
    }

    // overloaded function to find best match between single pointcloud and set of clusters using multi-objective optimization
    PointCloudPtr matchClustersMulti(PointCloud &cloud, PointCloudVec compares, int verbosity){

      std::vector<double> scores(compares.size()); // vector of scores, for debugging purposes
      std::vector<double> centroid_diffs(compares.size()), // vectors of differences 
                          volume_diffs(compares.size()), 
                          aspect_ratio_diffs(compares.size());

      std::vector<double> centroid_diffs_norm(compares.size()), // normalized vectors of differences 
                          volume_diffs_norm(compares.size()), 
                          aspect_ratio_diffs_norm(compares.size());                    

      double distance_x, distance_y, distance_z,
      volume1, volume2, 
      aspect_ratio1, aspect_ratio2,
      difference_x, difference_y, difference_z;

      double centroid_diffs_median, volume_diffs_median, aspect_ratio_diffs_median;
      
      // compare cloud to each cluster in compares                           
      for (int j=0; j<compares.size(); j++){

        // find the pca min bounding box for the ith cloud in clusters
        Eigen::Quaternionf quaternion1; 
        Eigen::Vector3f translation1, dimension1;
        Eigen::Matrix3f eigenvectors1;
        getPCABox(cloud, quaternion1, translation1, dimension1, eigenvectors1);

        // find the pca min bounding box for the jth cloud in compares
        Eigen::Quaternionf quaternion2; 
        Eigen::Vector3f translation2, dimension2;
        Eigen::Matrix3f eigenvectors2;
        getPCABox(*compares[j], quaternion2, translation2, dimension2, eigenvectors2);

        // calculate the objective differences for each pair 
        // term1 - position of centroid
        distance_x=translation1[0]-translation2[0];
        distance_y=translation1[1]-translation2[1];
        distance_z=translation1[2]-translation2[2];
        centroid_diffs.at(j) = pow(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2), 1.0/2.0); // square root of sum of squared component distances between centroids - l
        // term2 - volume of bounding box
        volume1=dimension1[0]*dimension1[1]*dimension1[2];
        volume2=dimension2[0]*dimension2[1]*dimension2[2];
        volume_diffs.at(j) = std::abs(volume1-volume2);
        // term3 - aspect ratio of bounding box 
        aspect_ratio1=  dimension1.maxCoeff()/dimension1.minCoeff(); 
        aspect_ratio2=  dimension2.maxCoeff()/dimension2.minCoeff(); 
        aspect_ratio_diffs.at(j)= std::abs(aspect_ratio1 - aspect_ratio2); // square root of squared difference in aspect ratios - l

      }

      // find the median value for each objective 
      centroid_diffs_median=getMedian(centroid_diffs);
      volume_diffs_median=getMedian(volume_diffs);
      aspect_ratio_diffs_median=getMedian(aspect_ratio_diffs);
      
      // find pair with min sum objective difference using median normalized differences 
      double score, score_min;
      int j_min=0; // default value for the search index, in case it is not set

      // seed the minimization with the first set of differences 
      score_min=centroid_diffs_norm[j_min]+volume_diffs_norm[j_min]+aspect_ratio_diffs_norm[j_min];

      for(int j=0; j<compares.size(); j++){ // re-check each possible pair
        // normalize diffs by dividing by median difference for each objective 
        centroid_diffs_norm[j]=centroid_diffs[j]/centroid_diffs_median;
        volume_diffs_norm[j]=volume_diffs[j]/volume_diffs_median;
        aspect_ratio_diffs_norm[j]=aspect_ratio_diffs[j]/aspect_ratio_diffs_median;  

        // calculate the score as the sum of the normalized diffs 
        score=centroid_diffs_norm[j]+volume_diffs_norm[j]+aspect_ratio_diffs_norm[j];

        if (score<score_min){ // find the lowest score
          score_min=score;
          j_min=j;            // record the index of the lowest score

        }
      }          

      //match=compares[j_min];

      if(verbosity>1){ // show the diff values from the search 
        
        for (int j=0; j<scores.size(); j++){
          std::cout <<"centroid_diffs["<<j<<"]: "<<centroid_diffs[j]
                    <<", volume_diffs["<<j<<"]: "<<volume_diffs[j]
                    <<", aspect_ratio_diffs["<<j<<"]: "<<aspect_ratio_diffs[j]<<std::endl;
        }

        std::cout <<"centroid_diffs_median: "<<centroid_diffs_median
                  <<", volume_diffs_median:"<<volume_diffs_median
                  <<", aspect_ratio_diffs_median: "<<aspect_ratio_diffs_median<<std::endl; 
        std::cout<<" normalized with median" <<std::endl;
        
        for (int j=0; j<scores.size(); j++){
          std::cout <<"centroid_diffs_norm["<<j<<"]: "<<centroid_diffs_norm[j]
                    <<", volume_diffs_norm["<<j<<"]: "<<volume_diffs_norm[j]
                    <<", aspect_ratio_diffs_norm["<<j<<"]: "<<aspect_ratio_diffs_norm[j]<<std::endl;
        }
      
      }
      
      if(verbosity>0){ // show the results of the search after complete
        std::cout <<"cloud has "<< cloud.size()<< " points " 
                  <<" and match has "<<compares[j_min]->size()<< " points"<<std::endl;
                  
      }
      return compares[j_min];  // return the compare with the best score to matches
    }



    // function to return the median value of a std::vector, it seems like there would be a std method for this
    double getMedian(std::vector<double> vals){

      size_t size=vals.size();

      if (size==0){
        return 0; // size 0 vector has no median
      }else{
        std::sort(vals.begin(), vals.end());
        if(size%2==0){
          return (vals[size/2-1]+vals[size/2])/2;
        }else{
          return vals[size/2];
        }

      }

    }

    // modified function to find best 1 to 1 correlation between two sets of clusters
    // for now this assumes size of clusters is less than or equal to size of compares to ensure 1-1 correspondence
    // this version checks all n^2 matches before removing any from the compare set 
    PointCloudVec matchClusters2(PointCloudVec clusters, PointCloudVec compares, int verbosity){

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
          score_min=scoreClouds(*clusters[cluster_indices[0]], *compares[compare_indices[0]], verbosity);  // seed the search with the score of first pair before the outside loop
          
          it_min=cluster_indices.begin(); // default value for it_min in case it is not assigned in search
          for (it=cluster_indices.begin(); it != cluster_indices.end(); it++){ // for each cluster in clusters1 find best match from clusters2 
            
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
            <<"](size:"<<compares[original_indices[k]]->size()<<") which is now matches["<<k<<"] (size:"<<matches[k]->size()<<")"<<std::endl;
            std::cout<<"the comparison score between clusters["<<k<<"] and compares["<<original_indices[k]<<"] is "<<scores[k]<<std::endl;
          }
        }
      
      }else{       // compares has fewer clusters than clusters, empty return 
        std::cout<<"warning: ( clusters.size() <= compares.size() ) failed, no matches returned"<<std::endl;
      }

      //std::cout<<"matches contains "<<matches.size()<<" clusters after matching complete"<<std::endl;
      return matches;
    }

    // function to find the intersection cloud3 of clouds1 and cloud2 defined by the points in cloud 1 AND cloud 2
    // this is based on exact comparison and will not work for approximate cloud points 
    void getCloudIntersection(PointCloud &cloud1, PointCloud &cloud2, PointCloud &cloud3){

      for (int i=0; i<cloud1.size(); i++) { // add points to cluster cloud
        for (int j=0; j<cloud2.size(); j++){
          // check if all three coordinate values are the same
          if (cloud1.points[i].x==cloud2.points[j].x&&cloud1.points[i].y==cloud2.points[j].y&&cloud1.points[i].z==cloud2.points[j].z){ 
            cloud3.push_back(cloud1[i]); // add the shared point to the new cloud
          }
        }
      }
      std::cout<< "the intersection cloud has "<< cloud3.size() << " points" <<std::endl;
    }

   
    // function to find the cluster of clouds representing the intersection of two clusters, calls SeamDetection::getClusterIntersection()   
    void getClusterIntersection(PointCloudVec &clusters1, PointCloudVec &clusters2, PointCloudVec &clusters3, int thresh){

      PointCloudPtr cloud (new PointCloud);
      //PointCloudVec clusters;

      int k=0; // comparison counter (counts each time)
      for(int i=0; i<clusters1.size(); i++){ // for each cluster in clusters1

        for (int j=0; j<clusters2.size(); j++){ // compare with each cluster in clusters2

          getCloudIntersection(*clusters1[i], *clusters2[j], *cloud); // find the points in clusters1[i] AND clusters2[j]

          if (cloud->size()>thresh){ // check if the intersection passes a threshold
            std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "<<cloud->size()<<" points and will be added to the intersection cluster"<<std::endl;
            //clusters.push_back(cloud); // add the intersection to the cluster of intersections
            clusters3.push_back(cloud);
          }else{
            std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "<<cloud->size()<<" points and will NOT be added to the intersection cluster"<<std::endl;
          }
          cloud->clear(); // empty the tmp cloud for the next intersection
          k++;
        }
      }

      std::cout<<"there are "<<clusters3.size()<<" clouds in the cluster intersection"<< std::endl;
      //return clusters;
    }
    

    // function to find the cluster of clouds representing the intersection of two clusters, calls SeamDetection::getClusterIntersection()   
    PointCloudVec getClusterIntersection(PointCloudVec &clusters1, PointCloudVec &clusters2, int thresh){

      PointCloudPtr cloud (new PointCloud); // tmp memory for kth test intersection 
      PointCloudVec clusters;

      int k=0; // comparison counter (counts each time)
      for(int i=0; i<clusters1.size(); i++){ // for each cluster in clusters1

        for (int j=0; j<clusters2.size(); j++){ // compare with each cluster in clusters2

          getCloudIntersection(*clusters1[i], *clusters2[j], *cloud); // find the points in clusters1[i] AND clusters2[j]

          if (cloud->size()>thresh){ // check if the intersection passes a threshold
            std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "<<cloud->size()<<" points and will be added to the intersection cluster"<<std::endl;
                                             
            // allocate memory for the pointcloud to be stored and pointed to by the new PointCloudVec  (vector of pointcloud pointers)
            PointCloudPtr cluster (new PointCloud);
            pcl::copyPointCloud(*cloud, *cluster); // make a copy to avoid the clear below

            // check multiple add here ... intersection 'cluster' is unique, new clusters should not have repeat entries... check on this  
            clusters.push_back(cluster); // add the intersection to the cluster of intersections
            //std::cout<<"the added cluster has "<<clusters[clusters.size()-1]->size()<<" points"<<std::endl; // the push is working....
            std::cout<<"the added cluster has "<<cluster->size()<<" points"<<std::endl;
            //

          }else{
            std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "<<cloud->size()<<" points and will NOT be added to the intersection cluster"<<std::endl;
          }
          cloud->clear(); // empty the tmp cloud for the next intersection, is this clear wiping both??? YES INDEED ! BUG IS HERE!
          std::cout<<"the added cluster has "<<clusters[clusters.size()-1]->size()<<" points after the clear"<<std::endl;
          k++;
        }
      }

      std::cout<<"there are "<<clusters.size()<<" clouds in the cluster intersection"<< std::endl;
      return clusters;
    }



    // function to merge a vector of pointclouds into a single pointcloud
    void mergeClusters(PointCloudVec &clusters, PointCloud &output){

      for (int i=0; i<clusters.size(); i++){
      
        for (int j=0; j<clusters[i]->size(); j++){
          output.push_back(clusters[i]->points[j]);
        }
      
      }

      std::cout<< "the merged cloud has "<< output.size() << " points" <<std::endl;
    }

    
    // overloaded function to merge a vector of pointclouds and return pointer to single pointcloud 
    PointCloudPtr mergeClusters(PointCloudVec &clusters){

      PointCloudPtr output (new PointCloud);

      for (int i=0; i<clusters.size(); i++){
      
        for (int j=0; j<clusters[i]->size(); j++){
          output->push_back(clusters[i]->points[j]);
        }
      
      }

      std::cout<< "the merged cloud has "<< output->size() << " points" <<std::endl;
      return output;
    }

    
    // function to publish a single cloud 
    void publishCloud(PointCloud &cloud, std::string topic){
      std::cout<<"|---------- SeamDetection::publishCloud - publishing single cloud ----------|"<<std::endl;

      // advertise a new topic and publish a msg each time this function is called
      pub_clouds.push_back(node.advertise<PointCloud>(topic, 0, true));
      
      cloud.header.frame_id = "base_link";

      pub_clouds[pub_clouds.size()-1].publish(cloud);

      ros::spinOnce();

    }


    // function to publish a vector of PointClouds representing clusters 
    void publishClusters(PointCloudVec &clusters, std::string prefix){
      std::cout<<"|---------- SeamDetection::publishClusters - publishing clusters ----------|"<<std::endl;
        
      for (int i=0; i<clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster from euclidean cluster extraction
        std::stringstream name;
        name << prefix << i;
        pub_clusters.push_back(node.advertise<PointCloud>(name.str(), 0, true));
        clusters[i]->header.frame_id = "base_link";
        pub_clusters[pub_idx].publish(clusters[i]);
        pub_idx++;
      }
      
      ros::spinOnce();
     
    }

    // attributes

    // pointcloud pointers
    PointCloud *training_input, *training_downsampled, *training_transformed, *training_bounded; 
    PointCloud *test_input, *test_downsampled, *test_transformed, *test_bounded; 
    //PointCloudPtr test_target;


    // other parameters from the config file (these do not need to public)
    bool auto_bounds=0;
    bool save_output, translate_output, automatic_bounds, use_clustering, new_scan, transform_input;
    std::string package_path, training_path, test_path, output_path, training_file, test_file, output_file; 
   
    double bounding_box[6];
    Eigen::Vector3f pre_rotation, pre_translation;

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
  ros::init(argc,argv,"seam_detection");
  
  // instantiate an object sd from the SeamDetection class
  SeamDetection sd;
 
  // Step 0 - load required parameters from ROS param server, modify values in seam-detection.yaml
  sd.loadConfig(); 
 

  // [Steps 1-3] - use 'training' image of target object on clean table

  // Step 1 - load the 'training' pointcloud from pcd file
  sd.loadCloud(sd.training_file, *sd.training_input);

  // Step 1.5 - perform voxel-downsampling, pre-transformation, and bounding-box on the training cloud
  double voxel_size=0.001; // voxel leaf size for downsampling
  sd.downsampleCloud(*sd.training_input, *sd.training_downsampled, voxel_size); 
  sd.transformCloud(*sd.training_downsampled, *sd.training_transformed, sd.pre_rotation, sd.pre_translation);
  sd.boundCloud(*sd.training_transformed, *sd.training_bounded, sd.bounding_box);
  
  // show the input training clouds in rviz
  sd.publishCloud(*sd.training_input, "/training_input"); 
  sd.publishCloud(*sd.training_downsampled, "/training_downsampled");
  sd.publishCloud(*sd.training_transformed, "/training_transformed"); 
  sd.publishCloud(*sd.training_bounded, "/training_bounded");
 
  // Step 2 - extract clusters from training cloud using euclidean and color algorithms
  PointCloudVec training_euclidean_clusters, training_color_clusters;
  training_euclidean_clusters=sd.extractEuclideanClusters(*sd.training_bounded, 200, 100000, 0.01); // preform Euclidean cluster extraction
  training_color_clusters=sd.extractColorClusters(*sd.training_bounded, 200, 10, 6, 5);             // preform Color Based Region Growing cluster extraction
  std::cout<<"training_euclidean_clusters size:"<<training_euclidean_clusters.size()<<std::endl;
  std::cout<<"training_color_clusters size:"<<training_color_clusters.size()<<std::endl;

  // Step 3 - choose proper euclidean clusters and color clusters using correlation routine between training euclidean and training color 
  int debug_level=1; // controls debug printing, 0-no print, 1-print search results, 2-print search data and search results 
  PointCloudVec training_matches; // keep in mind that this vector contains pointers to the original clusters data, no data copies made
  training_matches=sd.matchClustersMulti(training_euclidean_clusters, training_color_clusters, debug_level); 
  
  PointCloudPtr training_target;                    // use a pointer to the pointcloud data, no data copy required
  training_target=training_euclidean_clusters[0];   // assume the largest euclidean cluster is the target cluster
  //training_target=training_color_clusters[0];   // assume the largest color cluster is the training cluster
  // consider adding merge or something else here 

  // show the extracted 'test' clusters in rviz
  sd.publishClusters(training_euclidean_clusters, "/training_euclidean_cluster"); // show the euclidean and color based clusters  
  sd.publishClusters(training_color_clusters, "/training_color_cluster");         // for the training cloud  
  sd.publishClusters(training_matches, "/training_match");


  // [Steps 4-7] - use 'test' image of target object on cluttered table
  
  // Step 4 - load the 'test' pointcloud from pcd file (this is the cluttered table)
  sd.loadCloud(sd.test_file, *sd.test_input);
  
  // Step 5 - perform voxel-downsampling, pre-transformation, and bounding-box on the test cloud (same params used as in step 1.5)
  sd.downsampleCloud(*sd.test_input, *sd.test_downsampled, voxel_size); 
  sd.transformCloud(*sd.test_downsampled, *sd.test_transformed, sd.pre_rotation, sd.pre_translation);
  sd.boundCloud(*sd.test_transformed, *sd.test_bounded, sd.bounding_box);
 
  // show the input test clouds in rviz
  sd.publishCloud(*sd.test_input, "/test_input"); // show the input test and modified test clouds in rviz
  sd.publishCloud(*sd.test_downsampled, "/test_downsampled");
  sd.publishCloud(*sd.test_transformed, "/test_transformed"); 
  sd.publishCloud(*sd.test_bounded, "/test_bounded");

  // Step 6 - extract clusters from test cloud using euclidean and color algorithms
  PointCloudVec test_euclidean_clusters, test_color_clusters;
  test_euclidean_clusters=sd.extractEuclideanClusters(*sd.test_bounded, 200, 100000, 0.01); // preform Euclidean cluster extraction
  test_color_clusters=sd.extractColorClusters(*sd.test_bounded, 200, 10, 6, 5); // preform Color Based Region Growing cluster extraction
  
  std::cout<<"test_euclidean_clusters size:"<<test_euclidean_clusters.size()<<std::endl;
  std::cout<<"test_color_clusters size:"<<test_color_clusters.size()<<std::endl;

  // Step 7 - correlate test euclidean clusters to test color clusters 
  PointCloudVec test_matches;
  test_matches=sd.matchClustersMulti(test_euclidean_clusters, test_color_clusters, debug_level); 

  sd.publishClusters(test_euclidean_clusters, "/test_euclidean_cluster"); // show the euclidean and color based clusters  
  sd.publishClusters(test_color_clusters, "/test_color_cluster");         // for the test cloud  
  sd.publishClusters(test_matches, "/test_match");

  // [Steps 8 ... ] - compare 'training' target from steps 1-3 to correlated 'test' clusters from steps 4-7 
  // to find the target object in the test case 
  PointCloudPtr test_target;
  test_target=sd.matchClustersMulti(*training_target, test_euclidean_clusters, debug_level); 
  
  std::cout <<"the training target (size:"<<training_target->size()
            <<") is correlated with test_euclidean_clusters (size: "<<test_target->size()<<")"<<std::endl; 

  /*  // merge cloud example
  PointCloudPtr euclidean_merged (new PointCloud);
  sd.mergeClusters(euclidean_clusters, *euclidean_merged);
  std::cout<<"euclidean_merged has "<<euclidean_merged->size()<<" points"<< std::endl;
  */

  /* // cloud intersection example
  PointCloudPtr intersection_cloud (new PointCloud); // testing intersection method
  sd.getCloudIntersection(*euclidean_clusters[0], *color_clusters[0], *intersection_cloud);
  std::cout<<"intersection_cloud has "<<intersection_cloud->size()<<" points"<<std::endl;
  sd.publishCloud(*intersection_cloud, "/intersection_cloud"); // show in rviz
  */

  /*
  PointCloudVec intersection_clusters;
  intersection_clusters=sd.getClusterIntersection(euclidean_clusters, color_clusters, 500); 
  
  std::cout<<"intersection_clusters has "<<intersection_clusters.size()<<" clouds"<<std::endl;
  std::cout<<"intersection_clusters[0] has "<<intersection_clusters[0]->size()<<" points"<<std::endl;
  
  sd.publishClusters(euclidean_clusters, "/euclidean_cluster"); // show the euclidean and color based clusters  
  sd.publishClusters(color_clusters, "/color_cluster");           
  sd.publishClusters(intersection_clusters, "/intersection_cluster");
  */
   
  ros::spin();

  return 0;
}

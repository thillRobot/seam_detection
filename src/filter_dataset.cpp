/*
Seam Detection, FilterDataset
This source contains the cpp class FilterDataset for this project
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
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry> 

#include <pcl/common/common.h>
#include <pcl/pcl_config.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "ros/package.h"
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "cloudfilter.h"
#include "cloudutils.h"

namespace bf = boost::filesystem;
using namespace std::chrono_literals;

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

// PCL PointClouds with XYZ RGB Normal Points
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

// Vector of PointClouds
// aligned_allocator - STL compatible allocator to use with types requiring a non-standard alignment 
typedef std::vector < PointCloud::Ptr, Eigen::aligned_allocator < PointCloud::Ptr > > PointCloudVec;
typedef std::vector < PointCloudNormal::Ptr, Eigen::aligned_allocator < PointCloudNormal::Ptr > > PointCloudNormalVec;

class FilterDataset {

  public:

    // PUBLIC functions 
    
    // default constructor
    FilterDataset(): rate(5), pub_idx(0) { // ROS::Rate rate(5) is in intializer list
      
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"|---------- FilterDataset v1.9 ----------|"<<std::endl;
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

      // allocate memory for input pointcloud member
      input_cloud = new PointCloud; 

      // find the path to the this package (seam_detection)
      package_path = ros::package::getPath("seam_detection");
  
    }
  

    // function to load the config file(yaml) to pick the data files and set parameters 
    int loadConfig(void){

      std::cout<<"|---------- FilterDataset::LoadConfig - loading configuration file ---------|"<<std::endl;

      // get boolen parameters 
      node.getParam("save_bag_clouds", save_bag_clouds);
      node.getParam("save_dir_clouds", save_dir_clouds);
      node.getParam("auto_bounds", auto_bounds);

      // get parameters that contain strings  
      //node.getParam("input_file", input_file);    
      //node.getParam("output_file", output_file);
      node.getParam("input_dir", input_dir);    
      node.getParam("output_dir", output_dir);
 
      node.getParam("input_bag", input_bag);
      node.getParam("output_bag_dir", output_bag_dir);
      node.getParam("num_bag_clouds", num_bag_clouds);     

      // generate absolute file paths to inputs (does this belong here?)
      input_path=package_path+'/'+input_dir;
      output_path=package_path+'/'+output_dir;
            
      input_bag_path=package_path+'/'+input_bag;
      output_bag_path=package_path+'/'+output_bag_dir;

      // get parameters that contain doubles 
      node.getParam("voxel_size", voxel_size);
  
      node.getParam("publish_bag_clouds", publish_bag_clouds);
      node.getParam("publish_dir_clouds", publish_dir_clouds);
      
      node.getParam("use_bag_tfs", use_bag_tfs);
      node.getParam("broadcast_bag_tfs", broadcast_bag_tfs);
      node.getParam("use_config_tfs", use_config_tfs);
      
      node.getParam("transform_bag_clouds", transform_bag_clouds);
      node.getParam("bound_bag_clouds", bound_bag_clouds);
      node.getParam("smooth_bag_clouds", smooth_bag_clouds);
      node.getParam("downsample_bag_clouds", downsample_bag_clouds);
      
      node.getParam("transform_dir_clouds", transform_dir_clouds);
      node.getParam("bound_dir_clouds", bound_dir_clouds);
      node.getParam("smooth_dir_clouds", smooth_dir_clouds);
      node.getParam("downsample_dir_clouds", downsample_dir_clouds);

      // rotation and translation parameters from camera to fixed frame  
      std::vector<double> pre_rotation_vec, pre_translation_vec;
      node.getParam("pre_rotation",  pre_rotation_vec);
      node.getParam("pre_translation",  pre_translation_vec);
      for(unsigned i=0; i < pre_rotation_vec.size(); i++){
        pre_rotation[i]=pre_rotation_vec[i]; // copy from std vector to eigen vector3f 
        pre_translation[i]=pre_translation_vec[i]; 
      }
         
      // parameters that contain vectors of doubles
      //std::vector<double> bounding_box;
      node.getParam("bounding_box",  bounding_box);
      std::cout<<"bounding_box size: "<< bounding_box.size()<<std::endl;      
      //for(unsigned i=0; i < bounding_box_vec.size(); i++){
      //  bounding_box[i]=bounding_box_vec[i]; // copy from vector to array 
      //}
      node.getParam("auto_bounds",  auto_bounds);
      std::cout<<"bounding_box size: "<< bounding_box.size()<<std::endl;
      return 0;
   
    }
    

    // templated function to load pcl::PointCloud<point_t> from PCD file as defined in config
    template <typename point_t>
    int loadCloud(pcl::PointCloud<point_t> &input, std::string file) {

      std::cout<<"|---------- FilterDataset::LoadCloud - loading PCD file ----------|"<<std::endl;

      std::string path;
      //path=package_path+"/"+input_dir+"/"+file;
      path=file;

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
    int saveCloud(pcl::PointCloud<point_t> &output, std::string file){
      // the name output here is very confusing, consider changing this
      std::cout<<"|---------- FilterDataset::SaveCloud - saving PCD file ----------|"<<std::endl;

      std::string path;
      //path=package_path+"/"+output_dir+"/"+file;
      path=file; 
      // save filtered cloud 
      
      std::cout << "Saving output pointcloud file: " << path << std::endl;
      if (  pcl::io::savePCDFileASCII(path, output) == -1)
      {
        std::cout<<"Failed to save ouput pointcloud file: "<< path <<std::endl;
        return (-1);
      }
      std::cout << "Saved "<<output.width * output.height << " data points to output pointcloud file: "<< path <<std::endl;
      return 0;  
    } 
    
    
    // function to load pcd files (and frames?) from bag file
    PointCloudVec filterCloudBag(int num_clouds ){
      
      std::cout<<"|---------- FilterDataset::filterCloudBag - loading pointclouds from bag file ----------|"<<std::endl;
      
      rosbag::Bag bag;
      bag.open(input_bag_path, rosbag::bagmode::Read);

      std::string cloud_topic="camera/depth/color/points";
      std::string tf_topic="tf";

      std::vector<std::string> topics;
      topics.push_back(cloud_topic);
      topics.push_back(tf_topic);

      rosbag::View view(bag, rosbag::TopicQuery(topics));
           
      if (~bf::exists(output_bag_path)){
        bf::create_directory(output_bag_path);
        std::cout<<"_filtered directory created for output files from bag"<<std::endl;
      }
      //int nmax=20; // max clouds to process from bag  
     
      std::string in_path = bf::canonical(input_bag_path).string();  // input_bag_path from config
      // parse the input file name and create the modified output filename
      std::vector<std::string> strs;
      boost::split(strs,in_path,boost::is_any_of("/."));

      std::string in_file = strs[strs.size()-2];   
      std::string out_path;  

   //   std::cout<<"input bag file path: " <<in_path << std::endl;
   //   std::cout<<"output bag path: "<<output_bag_path<< std::endl;
   //   std::cout<<"output bag path prefix: "<<out_path<< std::endl;
      
      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::Transform T_camera_tripod, T_tripod_base, T_camera_base, T_base_camera, T_base_map, T_base_world,
                               T_camera_mount, T_mount_base, T_T6_base, T_T6_world;              
      geometry_msgs::TransformStamped tf_camera_base, tf_base_camera, tf_base_map, tf_T6_base, 
                                      tf_base_world, tf_T6_world, tf_map_world;      

      PointCloudVec bag_clouds;
      //use this as the working copy of the training cloud
      bool chain_complete=false;
     // std::cout<<"chain_complete: "<< chain_complete<< std::endl; 
      
      //tf::TransformListener listener;
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      
      tf2_msgs::TFMessage::Ptr transform;          

      std::cout<<"loading bag of clouds"<<std::endl;
      int idx=0;
      int cloud_idx=0;
      foreach(rosbag::MessageInstance const m, view){
        
        if (use_bag_tfs){   
          transform = m.instantiate<tf2_msgs::TFMessage>();
          //std::cout << "Received message on topic: " << m.getTopic() << std::endl; 
          if (transform!=NULL && !transform->transforms.empty()){
            //std::cout<<"pointer not null and transforms not empty"<<std::endl;
               
            for(const auto& tf : transform ->transforms){
              
              if (broadcast_bag_tfs){
                br.sendTransform(tf);  // broadcast the transforms to be used in rviz              
              }
 
              if (!strcmp(tf.header.frame_id.c_str(), "world") && !strcmp(tf.child_frame_id.c_str(), "T6")){
                //std::cout<<"tf recieved"<<std::endl;
                //std::cout<<"frame: "<<tf.header.frame_id<<std::endl;
                //std::cout<<"child frame: "<<tf.child_frame_id<<std::endl;
                
                tf_T6_base.transform=tf.transform;        // use the transform from tf (world->T6) and
                tf_T6_base.header.frame_id="base_link";   // re-broadcast as the transform fot tf (base_link-T6)
                tf_T6_base.child_frame_id="T6";            
                tf_T6_base.header.stamp=ros::Time::now();
                //chain_complete=true;
              } 
            
            }  
            
            // broadcast the bag transform from the end effector to the base of the robot 
            //if (chain_complete){
              //if(broadcast_bag_tfs){
            br.sendTransform(tf_T6_base);
              //}
              
              //chain_complete=false; 
              
            //}
          }     
        }

        if(use_config_tfs){  // now do this section also because we want bag tfs and published tf from reconfigure_tf.cpp
          // the tf tree is completed by the separate node `reconfigure_tf.cpp`
          // listen to and use a published tf instead of the tf from the bag to tranform the pointcloud
          geometry_msgs::TransformStamped tf_camera_base;
          try{
            // lookup a transform, this contains the chain from the the camera to the base of the robot 
            tf_camera_base = tfBuffer.lookupTransform("base_link", "camera_link", ros::Time(0)); 
            //std::cout<<"transform heard on bag loop iteration "<<idx<<std::endl;
            
            // inverse transfrom is not used, just showing that it is available
            tf_base_camera = tfBuffer.lookupTransform("camera_link", "base_link", ros::Time(0)); 
            //std::cout<<"inverse transform heard on bag loop iteration "<<idx<<std::endl;
            
            // convert tranform components to eigen objects to be used in pointcloud transformation 
            tf::vectorMsgToEigen(tf_camera_base.transform.translation, camera_base_translation);
            tf::quaternionMsgToEigen(tf_camera_base.transform.rotation, camera_base_rotation); 
          }
          catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
          }                   

        }
       
        sensor_msgs::PointCloud2::Ptr cloud = m.instantiate<sensor_msgs::PointCloud2>();          
        if (cloud != NULL && cloud_idx<num_clouds){
          // convert to a PCL pointcloud
            
          //PointCloud::Ptr bag_cloud (new PointCloud); // allocate memory for the loaded cloud      
          pcl::fromROSMsg(*cloud, *input_cloud); 
          std::cout<<"After converting to PCL pointcloud, the cloud has "<<input_cloud->size()<<" points"<<std::endl;
          
          // allocate memory for output of filtering process, to be stored (referenced) in pointcloud vec         
          PointCloud::Ptr filtered_cloud (new PointCloud);        
          
          // process pointcloud through series of filters
          filterCloud(*input_cloud, *filtered_cloud, 
                      transform_bag_clouds, 
                      bound_bag_clouds, 
                      smooth_bag_clouds, 
                      downsample_bag_clouds);
       
          bag_clouds.push_back(filtered_cloud);
          
          // optionally show the clouds in rviz  
          if (publish_bag_clouds){
            publishCloud(*input_cloud, "input_cloud", "base_link"); // publish from fixed frame to avoid tf time issues 
            publishCloud(*filtered_cloud, "filtered_cloud", "base_link");  
          }
         
          std::string out_file = in_file+std::to_string(cloud_idx)+".pcd"; 
          out_path=output_bag_path+"/"+out_file; // output_bag_path from config
          
          if(save_bag_clouds){  
            saveCloud(*filtered_cloud, out_path);
          }else{
            std::cout<<"pointcloud not saved, SAVE_OUTPUT false"<<std::endl;
          }

          cloud_idx++;
        }   
      
       //std::cout<<"bag topic loop counter: "<<idx<<std::endl;
       idx++;
      }
      std::cout<<"bag_clouds has "<<bag_clouds.size()<<" clouds"<<std::endl;
      bag.close();

      return bag_clouds;
    }

    
 
    // function to print contents of a geometry messages transform, used for debugging
    void printTransform(geometry_msgs::Transform transform, std::string name){

      std::cout<<std::endl<<"geometry_msgs::Transform "<<name<<std::endl; 
      std::cout<<"translation Vector: "<<std::endl;
      std::cout<<"X: "<<transform.translation.x<<std::endl;
      std::cout<<"Y: "<<transform.translation.y<<std::endl;
      std::cout<<"Z: "<<transform.translation.z<<std::endl;

      std::cout<<"rotation quaternion: "<<std::endl;
      std::cout<<"X: "<<transform.rotation.x<<std::endl;      
      std::cout<<"Y: "<<transform.rotation.y<<std::endl;
      std::cout<<"Z: "<<transform.rotation.z<<std::endl;
      std::cout<<"W: "<<transform.rotation.w<<std::endl;

    }   
    

    // function to print the contens of a tf2 quaternion, used for debugging
    void printQuaternion(tf2::Quaternion quaternion, std::string name){

      std::cout<<std::endl<<"geometry_msgs::Quaternion "<<name<<std::endl; 
      std::cout<<"rotation quaternion: "<<std::endl;
      std::cout<<"X: "<<quaternion.getAxis()[0]<<std::endl;      
      std::cout<<"Y: "<<quaternion.getAxis()[1]<<std::endl;
      std::cout<<"Z: "<<quaternion.getAxis()[2]<<std::endl;
      std::cout<<"W: "<<quaternion.getW()<<std::endl;

    }
  
    // function to filter an entire directory of pointclouds, results saved as output_path/*_filtered.pcd
    PointCloudVec filterCloudDir(std::string in){
       
      std::cout<<"|---------- FilterDataset::filterCloudDir - loading PCD files by directory ----------|"<<std::endl;

      PointCloudVec dir_clouds;
      // Load Input Directory with boost::filesystem
      //std::cout<<"filter input_path:"<<filter.input_path<<std::endl;
      boost::filesystem::path dir(in);
      
      try{
        
        if (bf::exists(dir)){
          
          if (bf::is_regular_file(dir)){
            
            std::cout << dir << " size is " << bf::file_size(dir) << std::endl;     
     
          }else if (bf::is_directory(dir)){
           
            if (~bf::exists(output_path)){
              bf::create_directory(output_path);
              std::cout<<"_filtered directory created for output files"<<std::endl;
            }

            std::cout << dir << " is a directory containing:"<< std::endl;
            int i=0;
            for (bf::directory_entry& x : bf::directory_iterator(dir)){
              std::string in_path = bf::canonical(x.path()).string();
              // parse the input file name and create the modified output filename
              std::vector<std::string> strs;
              boost::split(strs,in_path,boost::is_any_of("/."));

              std::string input_file = strs[strs.size()-2];
              std::string output_file = input_file+"_filtered.pcd";
             
              std::string out_path;
              out_path=output_path+"/"+output_file; // output path from config

              std::cout<<"input file["<<i<<"] path: " <<input_path << std::endl;
              std::cout<<"output file["<<i<<"] path: "<<out_path<< std::endl;
              if (in_path.find(".pcd")!=std::string::npos){          
                
                PointCloud::Ptr filtered_cloud (new PointCloud);          
                // load the pointcloud from pcd file
                loadCloud(*input_cloud, in_path);

                // process pointcloud through series of filters
                filterCloud(*input_cloud, *filtered_cloud, 
                            transform_dir_clouds, 
                            bound_dir_clouds, 
                            smooth_dir_clouds, 
                            downsample_dir_clouds);                
                
                // add to a vector of clouds to return
                dir_clouds.push_back(filtered_cloud); 
                std::cout<<"after filtering, there are "<<filtered_cloud->size()<<" points in the cloud"<< std::endl;          
                
                // show the filtered cloud each iteration
                if (publish_dir_clouds){
                  publishCloud(*input_cloud, "input_cloud", "map");  
                  publishCloud(*filtered_cloud, "filtered_cloud", "map");
                }
                
                // save the processed output cloud to PCD file
                if (save_dir_clouds){
                  saveCloud(*filtered_cloud, out_path);         
                }else{
                  std::cout<<"pointcloud not saved, SAVE_OUTPUT false"<<std::endl;
                }
                
                i++;
              }else{
                std::cout<<"skipping non PCD file"<<std::endl;
              }
            }   

          }else{
            std::cout << dir << " exists, but is not a regular file or directory"<<std::endl;
          }
       
        }else{
          std::cout << dir << " does not exist"<<std::endl;
        }
      
      }catch (const bf::filesystem_error& ex){
        std::cout << ex.what() << std::endl;
      }
      return dir_clouds;
    }
    
    
    // templated function to publish a single pcl::PointCloud<point_t> as a ROS topic 
    // this template is confusing, should be fixed 
    template <typename point_t>
    void publishCloud(point_t &cloud, std::string topic, std::string frame){
      std::cout<<"|---------- FilterDataset::publishCloud - publishing single cloud ----------|"<<std::endl;

      // advertise a new topic and publish a msg each time this function is called
      pub_clouds.push_back(node.advertise<pcl::PointCloud<point_t>>(topic, 0, true));
      
      cloud.header.frame_id = frame; // reference to show cloud in rviz

      pub_clouds[pub_clouds.size()-1].publish(cloud);

      ros::spinOnce();

    }


    // function to publish a vector of PointClouds representing clusters as a ROS topic
    void publishClouds(PointCloudVec &clusters, std::string prefix, std::string frame){
      std::cout<<"|---------- FilterDataset::publishClusters - publishing clusters ----------|"<<std::endl;
        
      for (int i=0; i<clusters.size(); i++){
        // advertise a topic and publish a msg for each cluster in clusters
        std::stringstream name;
        name << prefix << i;
        pub_clusters.push_back(node.advertise<PointCloud>(name.str(), 0, true));
        clusters[i]->header.frame_id = frame;
        pub_clusters[pub_idx].publish(clusters[i]);
        pub_idx++;
      }
      
      ros::spinOnce();
    }


    // function to publish a vector of PointClouds with normals representing clusters as a ROS topic
    void publishClusters(PointCloudNormalVec &clusters, std::string prefix){
      std::cout<<"|---------- FilterDataset::publishClusters - publishing clusters ----------|"<<std::endl;
        
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
    void publishClustersT(const std::vector<typename pcl::PointCloud<point_t>::Ptr, 
                          Eigen::aligned_allocator<typename pcl::PointCloud<point_t>::Ptr> > &clusters, 
                          std::string prefix){
      std::cout<<"|---------- FilterDataset::publishClusters - publishing clusters ----------|"<<std::endl;
        
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

      for (int i=0; i<input.size(); i++) { // add points to cluster cloud
        output.push_back(input[i]);  
      } 
    
    }

     
    // function to apply series of filters to pointcloud
    void filterCloud(PointCloud &input, PointCloud &output, 
                     bool transforming,
                     bool bounding,
                     bool smoothing,
                     bool downsampling){

      PointCloud::Ptr cloud (new PointCloud);
      pcl::copyPointCloud(input, *cloud);        // this copy ensures that the input data is left unchanged

      CloudFilter filter;
      filter.loadConfig("filter_dataset");
      
      // apply series of filters to pointcloud
      //rigid transformation
      if(transforming){
        //transformCloud(*cloud, *cloud, base_camera_rotation, base_camera_translation); // inverse
        //transformCloud(*cloud, *cloud, camera_base_rotation, camera_base_translation);
        filter.transformCloud(*cloud, *cloud, camera_base_rotation, camera_base_translation);
      }
      // bounding box 
      if(bounding){ 
        //boundCloud(*cloud, *cloud, bounding_box);
        filter.boundCloud(*cloud, *cloud, bounding_box);
      }
      // moving least squares smoothing  
      if(smoothing){
        //smoothCloudT(*cloud, *cloud); // smooth is slow on dense clouds not surprise
        PointCloudNormal::Ptr cloud_smoothed (new PointCloudNormal);
        filter.smoothCloud(*cloud, *cloud_smoothed); // smooth is slow on dense clouds not surprise
        publishCloud(*cloud_smoothed, "cloud_smoothed", "map");
      }
      // voxel downsampling  
      if(downsampling){
        //downsampleCloud(*cloud, *cloud, voxel_size);
        filter.downsampleCloud(*cloud, *cloud, voxel_size);
      }          

      // testing extract polygonal prism
      filter.extractPolygonalPrism(*cloud);
      
      std::cout<<"after filtering there are "<<cloud->size()<<" points in the cloud"<< std::endl;
      pcl::copyPointCloud(*cloud, output); // this copy is avoided by filtering "output" directly 
    }

    
    // PUBLIC  attributes

    // pointcloud pointers // !this public member could cause problems!, dont use this
    //pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud;

    PointCloud *input_cloud; // *filtered_cloud; // this may need to go too... 
    
    // other parameters from the config file (these do not need to public)
    bool auto_bounds;
    bool save_bag_clouds, save_dir_clouds, 
         broadcast_bag_tfs, use_bag_tfs, use_config_tfs,
         publish_bag_clouds, publish_dir_clouds,  
         transform_bag_clouds, transform_dir_clouds,
         bound_bag_clouds, bound_dir_clouds,
         smooth_bag_clouds, smooth_dir_clouds,
         downsample_bag_clouds, downsample_dir_clouds; 
    int num_bag_clouds;

    std::string package_path, input_path, output_path, input_dir, output_dir, input_file, output_file, 
                input_bag, input_bag_path, output_bag_dir, output_bag_path;
 
    std::vector<double> bounding_box;
    Eigen::Vector3d pre_rotation, pre_translation;

    double voxel_size;

    // topic for generic cloud publisher
    std::string cloud_topic;

    Eigen::Vector3d camera_base_translation;
    Eigen::Quaterniond camera_base_rotation;
 
    Eigen::Vector3d base_camera_translation;
    Eigen::Quaterniond base_camera_rotation;

     
   private:

    // PRIVATE attributes

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
  ros::init(argc,argv,"filter_dataset");
  
  // instantiate a filter object 
  FilterDataset filter;
 
  // load ROS parameters, modify values in filter-cloud.yaml
  filter.loadConfig(); 

  // process pointcloud topics from bag file
  PointCloudVec bagclouds;  
  bagclouds=filter.filterCloudBag(filter.num_bag_clouds);
  // show in rviz
  //filter.publishClouds(bagclouds, "bag_cloud", "base_link");
  
  // process pcd files from directory 
  PointCloudVec dirclouds;
  dirclouds=filter.filterCloudDir(filter.input_path);
  // show in rviz
  //filter.publishClouds(dirclouds, "dir_cloud", "base_link");
  
  
  

  std::cout<<"filter_dataset completed"<<std::endl;
  ros::spin();

  return 0;
}

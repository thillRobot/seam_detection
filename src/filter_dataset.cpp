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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


namespace bf = boost::filesystem;
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

class FilterDataset {

  public:

    // PUBLIC functions 
    
    // default constructor
    FilterDataset(): rate(5), pub_idx(0) { // ROS::Rate rate(5) is in intializer list
      
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"|---------- FilterDataset v1.9 ----------|"<<std::endl;
      std::cout<<"|----------------------------------------|"<<std::endl;
      std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

      // allocate memory for pointclouds member attributes
      input_cloud = new PointCloud;   // this is not a good idea to use this name
      filtered_cloud = new PointCloud;

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
      //node.getParam("transform_input", transform_input);

      // get parameters that contain strings  
      //node.getParam("input_file", input_file);    
      //node.getParam("output_file", output_file);
      node.getParam("input_dir", input_dir);    
      node.getParam("output_dir", output_dir);
 
      node.getParam("input_bag", input_bag);
      node.getParam("output_bag_dir", output_bag_dir);
      
      // generate absolute file paths to inputs (does this belong here?)
      input_path=package_path+'/'+input_dir;
      output_path=package_path+'/'+output_dir;
            
      input_bag_path=package_path+'/'+input_bag;
      output_bag_path=package_path+'/'+output_bag_dir;

      // get parameters that contain doubles 
      node.getParam("voxel_size", voxel_size);

      // parameters that contain vectors of doubles
      std::vector<double> bounding_box_vec;
      node.getParam("bounding_box",  bounding_box_vec);
      for(unsigned i=0; i < bounding_box_vec.size(); i++){
        bounding_box[i]=bounding_box_vec[i]; // copy from vector to array 
      }
      
      node.getParam("transform_input", transform_input);
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
    int loadCloud(pcl::PointCloud<point_t> &input, std::string file) {

      std::cout<<"|---------- SeamDetection::LoadCloud - loading PCD file ----------|"<<std::endl;

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
      std::cout<<"|---------- SeamDetection::SaveCloud - saving PCD file ----------|"<<std::endl;

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
    
    
    // function to filter an entire directory of pointclouds, results saved as output_path/*_filtered.pcd
    int filterCloudDir(std::string in){
       
      std::cout<<"|---------- SeamDetection::filterCloudDir - loading PCD files by directory ----------|"<<std::endl;

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
                
                // load the pointcloud from pcd file
                loadCloud(*input_cloud, in_path);
                filterCloud(*input_cloud, *filtered_cloud);
                std::cout<<"after processing, there are "<<filtered_cloud->size()<<" points in the cloud"<< std::endl;          
                // show the smooth and downsamples each iteration
                //publishCloud(*smoothed_cloud, "/smoothed", "camera_link");
                publishCloud(*filtered_cloud, "/filtered", "camera_link");
                
                // save the processed output cloud to PCD file
                if (save_output){
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
      return 1;
    }


    // function to load pcd files (and frames?) from bag file
    PointCloudVec filterCloudBag(void){
      
      std::cout<<"|---------- SeamDetection::filterCloudBag - loading pointclouds from bag file ----------|"<<std::endl;
      
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
      int nmax=20; // max clouds to process from bag  
    
      std::string in_path = bf::canonical(input_bag_path).string();  // input_bag_path from config
      // parse the input file name and create the modified output filename
      std::vector<std::string> strs;
      boost::split(strs,in_path,boost::is_any_of("/."));

      std::string in_file = strs[strs.size()-2];
      //std::string out_file = in_file+"_filtered.pcd";
      
      std::string out_path;
      //out_path=output_bag_path+"/"+out_file; // output_bag_path from config
        
   //   std::cout<<"input bag file path: " <<in_path << std::endl;
   //   std::cout<<"output bag path: "<<output_bag_path<< std::endl;
   //   std::cout<<"output bag path prefix: "<<out_path<< std::endl;
      
      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::Transform T_camera_tripod, T_tripod_base, T_camera_base, T_base_map;              
      geometry_msgs::TransformStamped tf_camera_base, tf_base_map;      

      PointCloudVec bag_clouds;
      //use this as the working copy of the training cloud
      bool chain_complete=false;
     // std::cout<<"chain_complete: "<< chain_complete<< std::endl; 
      
      std::cout<<"loading bag of clouds"<<std::endl;
      int idx=0;
      int cloud_idx=0;
      foreach(rosbag::MessageInstance const m, view){
               
        //std::cout << "Received message on topic: " << m.getTopic() << std::endl; 
        tf2_msgs::TFMessage::Ptr transform = m.instantiate<tf2_msgs::TFMessage>();          
        if (transform!=NULL && !transform->transforms.empty()){
          //std::cout<<"pointer not null and transforms not empty"<<std::endl;
             
          for(const auto& tf : transform ->transforms){
            br.sendTransform(tf);  // broadcast the transforms to be used in rviz              
            //std::cout<<"tf header frame_id: "<<tf.header.frame_id<<std::endl;
            //std::cout<<"tf child_frame_id: "<<tf.child_frame_id<<std::endl;
            
            if (!strcmp(tf.header.frame_id.c_str(), "base_link")){
              T_tripod_base=tf.transform;              
            }
            if (!strcmp(tf.header.frame_id.c_str(), "tripod_link")){
              T_camera_tripod=tf.transform;              
              chain_complete=true; // this assumes a consistent broadcast order
              tf_camera_base.header.stamp=tf.header.stamp;  // copy the most recent timestamp from above
            }
            
          }  
       //   std::cout<<"chain_complete: "<< chain_complete<< std::endl; 
          if (chain_complete){
            // calculate the total translation vector as the sum of translation vectors
            T_camera_base.translation.x=T_tripod_base.translation.x+T_camera_tripod.translation.x;              
            T_camera_base.translation.y=T_tripod_base.translation.y+T_camera_tripod.translation.y;              
            T_camera_base.translation.z=T_tripod_base.translation.z+T_camera_tripod.translation.z;              
                          
            // calculate the total quaternion as the product of the quaternion vectors  
            // do not operate directly on the mgs, convert first
            tf2::Quaternion q_tripod_base, q_camera_tripod, q_camera_base;
            tf2::fromMsg(T_tripod_base.rotation, q_tripod_base); 
            tf2::fromMsg(T_camera_tripod.rotation, q_camera_tripod);          
            //q_camera_base=q_tripod_base*q_camera_tripod;    // multiply to combine quaternions   
            q_camera_base=q_camera_tripod*q_tripod_base;      // order matters 
            T_camera_base.rotation=tf2::toMsg(q_camera_base); // convert from tf2 to geometry msg
             
            // now put that in a stamped transform to be re-broadcasted in the tf topic
            //geometry_msgs::TransformStamped tf_camera_base;      
            tf_camera_base.header.frame_id="base_link";       // create new header info
            tf_camera_base.child_frame_id="camera_link_test"; // check loop closure in rviz to validate tf operations
           
            tf_camera_base.transform=T_camera_base;           // add the computed transform to the message 
            br.sendTransform(tf_camera_base);
             
            // add transform to stamped transform to be broadcasted 
            //printTransform(T_tripod_base, "T_tripod_base"); 
            //printTransform(T_camera_tripod, "T_camera_tripod");
            //printTransform(T_camera_base, "T_camera_base");
            chain_complete=false; 
          
            // convert the combined transformation to eigen to be used for transforming clouds
            tf::vectorMsgToEigen(T_camera_base.translation, camera_translation);// convert translation to eigen vector
            tf::quaternionMsgToEigen(T_camera_base.rotation, camera_rotation); // convert rotation to eigen quaternion
          
          }
          
        }     
        
        sensor_msgs::PointCloud2::Ptr cloud = m.instantiate<sensor_msgs::PointCloud2>();          
        if (cloud != NULL && cloud_idx<nmax){
          // convert to a PCL pointcloud
            
          PointCloud::Ptr bag_cloud (new PointCloud); // allocate memory for the loaded cloud      
          pcl::fromROSMsg(*cloud, *bag_cloud); 
          std::cout<<"After converting to PCL pointcloud, the cloud has "<<bag_cloud->size()<<" points"<<std::endl;

          publishCloud(*bag_cloud, "bag_cloud", "camera_link");  
 
          PointCloud::Ptr filtered_cloud (new PointCloud);          
          filterCloud(*bag_cloud, *filtered_cloud);          
          
          publishCloud(*filtered_cloud, "filtered_cloud", "base_link");  
          bag_clouds.push_back(bag_cloud);
          
          std::string out_file = in_file+std::to_string(cloud_idx)+".pcd"; 
          out_path=output_bag_path+"/"+out_file; // output_bag_path from config
            
          saveCloud(*filtered_cloud, out_path);
          cloud_idx++;
        }   
      
       //std::cout<<"bag topic loop counter: "<<idx<<std::endl;
       idx++;
      }
      std::cout<<"bag_clouds has "<<bag_clouds.size()<<" clouds"<<std::endl;
      bag.close();

      return bag_clouds;
    }

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
    
    void printQuaternion(tf2::Quaternion quaternion, std::string name){

      std::cout<<std::endl<<"geometry_msgs::Quaternion "<<name<<std::endl; 
      std::cout<<"rotation quaternion: "<<std::endl;
      std::cout<<"X: "<<quaternion.getAxis()[0]<<std::endl;      
      std::cout<<"Y: "<<quaternion.getAxis()[1]<<std::endl;
      std::cout<<"Z: "<<quaternion.getAxis()[2]<<std::endl;
      std::cout<<"W: "<<quaternion.getW()<<std::endl;

    }
   
    // templated function to publish a single pcl::PointCloud<point_t> as a ROS topic 
    template <typename point_t>
    void publishCloud(point_t &cloud, std::string topic, std::string frame){
      std::cout<<"|---------- SeamDetection::publishCloud - publishing single cloud ----------|"<<std::endl;

      // advertise a new topic and publish a msg each time this function is called
      pub_clouds.push_back(node.advertise<pcl::PointCloud<point_t>>(topic, 0, true));
      
      cloud.header.frame_id = frame; // reference to show cloud in rviz

      pub_clouds[pub_clouds.size()-1].publish(cloud);

      ros::spinOnce();

    }


    // function to publish a vector of PointClouds representing clusters as a ROS topic
    void publishClouds(PointCloudVec &clusters, std::string prefix, std::string frame){
      std::cout<<"|---------- SeamDetection::publishClusters - publishing clusters ----------|"<<std::endl;
        
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

      //std::cout<<"the point cloud input has "<< input.size()<< " points"<<std::endl;
      for (int i=0; i<input.size(); i++) { // add points to cluster cloud
        output.push_back(input[i]);  
      } 
      //std::cout<<"the point cloud output has "<< output.size()<< " points"<<std::endl;
    
    }


    // function to apply voxel downsampling to pointcloud 
    void downsampleCloud(PointCloud &input, PointCloud &output, double leaf_size){

      PointCloud::Ptr cloud (new PointCloud); 
      pcl::copyPointCloud(input, *cloud);        // this copy ensures that the input data is left unchanged
 
      // Apply Voxel Filter 
      if (leaf_size>0){
        pcl::VoxelGrid<PointT> vox;
        vox.setInputCloud (cloud); // operate directly on the output PointCloud pointer, removes need for copy below
        vox.setLeafSize (leaf_size, leaf_size, leaf_size); // use "001f","001f","0001f" or "none" to set voxel leaf size
        vox.filter (*cloud);
      }else{
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
     
      double box_length, box_width, box_height;
      box_length=0.25; // default auto_bounds, smart auto bounds not implemented
      box_width=0.25;     
      box_height=0.25;

//      if (auto_bounds){
//     
//        Eigen::Vector4f centroid;
//        Eigen::Vector4f min;
//        Eigen::Vector4f max;  
//
//        pcl::compute3DCentroid(*cloud, centroid);
//
//        box[0]=centroid[0]-box_length/2;  // xmin
//        box[1]=centroid[0]+box_length/2;  // xmax
//        box[2]=centroid[1]-box_width/2;   // ymin
//        box[3]=centroid[1]+box_width/2;   // ymax
//        box[4]=centroid[2]-box_height/2;  // zmin
//        box[5]=centroid[2]+box_height/2;  // zmax
//
//      }else{
//      }

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
        
      pcl::copyPointCloud(*cloud, output);
      std::cout<<"after bounding there are "<<output.size()<<"points in the cloud"<<std::endl;
    }


    // function to apply translation and rotation without scaling to PointCloud
    void transformCloud(PointCloud &input, PointCloud &output, Eigen::Vector3d rotation, Eigen::Vector3d translation){

      PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy of the training cloud
      pcl::copyPointCloud(input,*cloud);

      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      // Define a translation 
      transform.translation() << translation[0], translation[1], translation[2];
      // define three axis rotation&clouds (RPY)
      transform.rotate (Eigen::AngleAxisd (rotation[0], Eigen::Vector3d::UnitX()));
      transform.rotate (Eigen::AngleAxisd (rotation[1], Eigen::Vector3d::UnitY()));
      transform.rotate (Eigen::AngleAxisd (rotation[2], Eigen::Vector3d::UnitZ()));

      // Print the transformation
      //std::cout << transform_2.matrix() << std::endl;

      // Execute the transformation on working copy 
      pcl::transformPointCloud (*cloud, *cloud, transform); 
      // copy to the output cloud
      pcl::copyPointCloud(*cloud, output);
      
      std::cout<<"after transformation there are "<<output.size()<<" points"<<std::endl;
    }

    // overloaded function to apply translation and rotation without scaling to PointCloud
    void transformCloud(PointCloud &input, PointCloud &output, Eigen::Quaterniond rotation, Eigen::Vector3d translation){

      PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy of the training cloud
      pcl::copyPointCloud(input,*cloud);

      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      // Define a translation 
      transform.translation() << translation[0], translation[1], translation[2];
      
      transform.rotate(rotation);

      // Print the transformation
      //std::cout << transform_2.matrix() << std::endl;

      // Execute the transformation on working copy 
      pcl::transformPointCloud (*cloud, *cloud, transform); 
      // copy to the output cloud
      pcl::copyPointCloud(*cloud, output);
      
      std::cout<<"after transformation there are "<<output.size()<<" points"<<std::endl;
    }

    void smoothCloud(PointCloud &input, PointCloudNormal &output){

      PointCloud::Ptr cloud (new PointCloud);  //use this as the working copy for this function 
      pcl::copyPointCloud(input,*cloud);

      // Create a KD-Tree
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

      // Output has the PointNormal type in order to store the normals calculated by MLS
      //pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points; 
      // modify the function output pointcloud directly instead
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
      std::cout<<"after smoothing there are "<<output.size()<<"points in the cloud"<<std::endl;
    }

    // templated function for PCL moving least squares smoothing, normal data generated during this process
    template <typename point_t, typename point_normal_t> 
    void smoothCloudT(pcl::PointCloud<point_t> &input, pcl::PointCloud<point_normal_t> &output){
      
      //allocate memory and make copy to use as the working copy for this function 
      typename pcl::PointCloud<point_t>::Ptr cloud (new pcl::PointCloud<point_t>); 
      pcl::copyPointCloud(input,*cloud);
      
      std::cout<<"before smoothing there are "<<cloud->size()<<"points in the cloud"<<std::endl;
      // Create a KD-Tree
      typename pcl::search::KdTree<point_t>::Ptr tree (new pcl::search::KdTree<point_t>);

      pcl::MovingLeastSquares<point_t, point_normal_t> mls;

      mls.setComputeNormals (true);
      // Set parameters

      mls.setInputCloud (cloud);
      mls.setPolynomialOrder (2);
      mls.setSearchMethod (tree);
      mls.setSearchRadius (0.03);

      // Reconstruct
      mls.process (output);
      std::cout<<"after smoothing there are "<<output.size()<<"points in the cloud"<<std::endl;
   } 

    // function to apply filter pipeline to cloud
    void filterCloud(PointCloud &input, PointCloud &output){

      PointCloud::Ptr cloud (new PointCloud);
      pcl::copyPointCloud(input, *cloud);        // this copy ensures that the input data is left unchanged

      //rigid transformation, bounding-box, smoothing, and voxel-downsampling on the input cloud
      if(transform_input){
        transformCloud(*cloud, *cloud, camera_rotation, camera_translation);
      }

      boundCloud(*cloud, *cloud, bounding_box);
      //smoothCloudT(*cloud, *cloud); // smooth is slow on dense clouds not surprise
      downsampleCloud(*cloud, *cloud, voxel_size);
      std::cout<<"after filtering there are "<<cloud->size()<<" points in the cloud"<< std::endl;

      pcl::copyPointCloud(*cloud, output); // this copy is avoided by filtering "output" directly 
    }

    
    // PUBLIC  attributes

    // pointcloud pointers // !this public member could cause problems!, dont use this
    //pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud;

    PointCloud *input_cloud, *filtered_cloud; 
    
    // other parameters from the config file (these do not need to public)
    bool auto_bounds=0;
    bool save_output, translate_output, automatic_bounds, use_clustering, new_scan, transform_input;
    
    std::string package_path, input_path, output_path, input_dir, output_dir, input_file, output_file, 
                input_bag, input_bag_path, output_bag_dir, output_bag_path;
   
    double bounding_box[6];
    Eigen::Vector3d pre_rotation, pre_translation;

    double voxel_size;

    // topic for generic cloud publisher
    std::string cloud_topic;

    Eigen::Vector3d camera_translation;
    Eigen::Quaterniond camera_rotation;

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
  ros::init(argc,argv,"filter_cloud");
  
  // instantiate an object filter.from the SeamDetection class
  FilterDataset filter;
 
  // load parameters from ROS param server, modify values in filter-cloud.yaml
  filter.loadConfig(); 

  filter.input_file="table_01.pcd";
  //std::cout<<"input_path: "<< filter.input_path << std::endl;

  PointCloudVec clouds_in;
  
  // process pcd files in input_path/ and save in output_path/
  //filter.filterCloudDir(filter.input_path);

  // process pointcloud topics from bag file
  PointCloudVec clouds;  
  clouds=filter.filterCloudBag();

  filter.publishClouds(clouds, "bag_clouds", "camera_link");


  std::cout<<"filter_cloud completed"<<std::endl;
  ros::spin();

  return 0;
}

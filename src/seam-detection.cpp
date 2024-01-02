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
//#include <tf/TransformStamped.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;


class SeamDetection {

  public:

    // functions 
    
    // default constructor
    SeamDetection():rate(5) { // ROS::Rate rate(5) is in intializer list
      
      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"                     SeamDetection v1.9                             "<<std::endl;
      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

      // allocate memory for pointclouds
      cloud_input = new PointCloud;
      cloud_transformed = new PointCloud;
      cloud_bounded = new PointCloud;

      // find the path to the this package (seam_detection)
      package_path = ros::package::getPath("seam_detection");
  
    }
  

    // function to load the config file(yaml) to pick the data files and set parameters 
    int LoadConfig(void){

      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"         SeamDetection::LoadConfig - loading configuration file     "<<std::endl;
      std::cout<<"===================================================================="<<std::endl<<std::endl;

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

      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"       SeamDetection::LoadCloud - loading configuration file        "<<std::endl;
      std::cout<<"===================================================================="<<std::endl<<std::endl;

      node.getParam("seam_detection/input_file", input_file);
      input_path=package_path+'/'+input_file;

      // instantiate cloud pointer
      //PointCloud::Ptr cloud_input (new PointCloud); 
      PointCloud::Ptr cloud (new PointCloud);      // working copy for this routine

      std::cout << "Loading cloud input file:" << input_path << std::endl;
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_path, *cloud_input) == -1)
      {
          std::cout<<"Couldn't read cloud input file:"<<input_path;
          return (-1);
      }
      std::cout << "Loaded cloud input file: "<< input_path <<std::endl<<
        cloud_input->width * cloud_input->height << " Data points from "<< input_path << std::endl;
 
      return 0;  
    }


    // function to copy PointCloud with XYZRGB points
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
      pcl::PassThrough<PointT> pass; //cloud_input
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

    // function to publish a single cloud (not working for multiple calls, blocking error)
    void PublishCloud(PointCloud &cloud, std::string topic)
    {

      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"       SeamDetection::ShowCloud - preparing visualization           "<<std::endl;
      std::cout<<"===================================================================="<<std::endl<<std::endl;

      ros::Publisher pub = node.advertise<PointCloud> (topic, 1, true);

      //cloud_input->header.frame_id = "base_link";
      cloud.header.frame_id = "base_link";

      pub.publish(cloud);
      ros::spin();

    }

    // function to publish the input and other clouds to ROS for RVIZ 
    void PublishClouds()
    {

      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"       SeamDetection::ShowClouds - preparing visualization           "<<std::endl;
      std::cout<<"===================================================================="<<std::endl<<std::endl;


      //cloud_input->header.frame_id = "base_link";
      cloud_input->header.frame_id = "base_link";
      cloud_transformed->header.frame_id = "base_link";
      cloud_bounded->header.frame_id = "base_link";

      pub_input.publish(*cloud_input);
      pub_transformed.publish(*cloud_transformed);
      pub_bounded.publish(*cloud_bounded);
      
      ros::spin();

    }


    // attributes
    PointCloud *cloud_input, *cloud_transformed, *cloud_bounded;

    bool auto_bounds=0;
    bool save_output, translate_output, automatic_bounds, use_clustering, new_scan, transform_input;
    std::string package_path, input_path, output_path, target_path, input_file, output_file, target_file; 
   
    double bounding_box[6];
    Eigen::Vector3f pre_rotation, pre_translation;

    ros::NodeHandle node; // !!! many examples have node in 'private'
    ros::Rate rate;       // node and rate might be useful in 'public' ...


  private:

    // attributes
    const int ijk=0;

    ros::Publisher pub_input = node.advertise<PointCloud> ("/cloud_input", 1, true);
    ros::Publisher pub_transformed = node.advertise<PointCloud> ("cloud_transformed", 1, true);
    ros::Publisher pub_bounded = node.advertise<PointCloud> ("cloud_bounded", 1, true);

};


int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc,argv,"seam_detection");
  
  SeamDetection sd;
 
  sd.LoadConfig();

  sd.LoadCloud(sd.input_file);

  PointCloud::Ptr cloud_copy (new PointCloud);
  //PointCloud::Ptr cloud_transformed (new PointCloud);
  //PointCloud::Ptr cloud_bounded (new PointCloud);

  sd.CopyCloud(*sd.cloud_input, *cloud_copy);

  sd.TransformCloud(*cloud_copy, *sd.cloud_transformed, sd.pre_rotation, sd.pre_translation);
  
  sd.BoundCloud(*sd.cloud_transformed, *sd.cloud_bounded, sd.bounding_box);

  //sd.PublishCloud(*sd.cloud_input, "/cloud_input");
  //sd.PublishCloud(*cloud_transformed, "/cloud_transformed");
  //sd.PublishCloud(*cloud_bounded, "/cloud_bounded");

  sd.PublishClouds();

  // loop forever
  //while(ros::ok())
  //{

    //ros::spinOnce();
    //sd.rate.sleep();
  
  //}
  
  ros::spin();

  return 0;
}

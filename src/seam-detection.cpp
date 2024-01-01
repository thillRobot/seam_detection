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

    // default constructor
    SeamDetection(){
    
      std::cout<<"Seam Detection v1.9"<<std::endl;
    
      cloud_input = new PointCloud;

    }

    // attributes
    PointCloud *cloud_input;


    int LoadCloud(void){

      std::cout<<"===================================================================="<<std::endl;
      std::cout<<"       SeamDetection::LoadCloud : loading configuration file       "<<std::endl;
      std::cout<<"===================================================================="<<std::endl<<std::endl;

      // there is only one cmd line arg and it is the name of the config file
      // this is not true, the params are loaded in the launch file
      // read the config file(yaml) feild to pick the data files and set parameters

      // find the path to the this package (seam_detection)
      std::string packagepath = ros::package::getPath("seam_detection");
      std::string filepath = "pcd_images/ds435i_table_parts/table_part1_part2_02.pcd";
      std::string input_path;

      input_path=packagepath+'/'+filepath;

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


  private:

    const int ijk=0;

};



int main(int argc, char** argv)
{

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     SeamDetection v1.9                    "<<endl;
  std::cout<<"===================================================================="<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  SeamDetection sd;
 
  sd.LoadCloud();


  PointCloud::Ptr cloud_a (new PointCloud);
  PointCloud::Ptr cloud_b (new PointCloud);
  
  sd.CopyCloud(*cloud_a, *cloud_b);


  return 0;
}



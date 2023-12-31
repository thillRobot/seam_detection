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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;


int main(int argc, char** argv)
{

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     Seam Detection Class v1.7                    "<<endl;
  std::cout<<"===================================================================="<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                      Loading Configuration File                    "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // there is only one cmd line arg and it is the name of the config file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters 
  bool save_part1_segment, save_part2_segment;
  node.getParam("save_part1_segment", save_part1_segment);
  node.getParam("save_part2_segment", save_part2_segment);

  // parameters that contain strings  
  std::string source_path, target_path, source_file, target_file, part1_type; 
  node.getParam("source_file", source_file);
  source_path=packagepath+'/'+source_file;

  node.getParam("target_file", target_file);
  target_path=packagepath+'/'+target_file;

  node.getParam("part1_type",  part1_type);

  // parameters that contain doubles
  double voxel_leaf_size, ransac_norm_dist_wt, ransac_max_iter, ransac_dist_thrsh, ransac_k_srch,
         icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl;

  // parameters that contain vectors of doubles
  std::vector<double> xs, ys, zs, filter_box_vec, ransac_init_norm_vec, expected_results_vec, calibration_offset_vec, seam1_points_x_vec, seam1_points_y_vec, seam1_points_z_vec;
  double filter_box[6],ransac_init_norm[3],icp_params[4],expected_results[6],calibration_offset[6],seam1_points_x[4],seam1_points_y[4],seam1_points_z[4];
  
  node.getParam("filter_box",  filter_box_vec);
  for(unsigned i=0; i < filter_box_vec.size(); i++)
    filter_box[i]=filter_box_vec[i]; // copy from vector to array 
  node.getParam("voxel_leaf_size", voxel_leaf_size);

  node.getParam("ransac_norm_dist_wt",ransac_norm_dist_wt);  
  node.getParam("ransac_max_iter",ransac_max_iter);  
  node.getParam("ransac_dist_thrsh",ransac_dist_thrsh);  
  node.getParam("ransac_k_srch",ransac_k_srch);  
  node.getParam("ransac_init_norm",  ransac_init_norm_vec);
  for(unsigned i=0; i < ransac_init_norm_vec.size(); i++)
    ransac_init_norm[i]=ransac_init_norm_vec[i]; // copy from vector to array
  
  node.getParam("icp_max_corr_dist",icp_max_corr_dist);  // these four ICP parameters define the search
  node.getParam("icp_max_iter",icp_max_iter);
  node.getParam("icp_trns_epsl",icp_trns_epsl);
  node.getParam("icp_ecld_fitn_epsl",icp_ecld_fitn_epsl);

  node.getParam("expected_results",expected_results_vec);  // these four ICP parameters define the search
  node.getParam("calibration_offset",calibration_offset_vec);  // these four ICP parameters define the search
  for(unsigned i=0; i < expected_results_vec.size(); i++){
    expected_results[i]=expected_results_vec[i]; // copy into an array 
    calibration_offset[i]=calibration_offset_vec[i]; // copy into an array 
  }
  
  node.getParam("seam1_points_x",seam1_points_x_vec);
  node.getParam("seam1_points_y",seam1_points_y_vec);
  node.getParam("seam1_points_z",seam1_points_z_vec);
  for(unsigned i=0; i < seam1_points_x_vec.size(); i++){
    seam1_points_x[i]=seam1_points_x_vec[i]; // copy into arrays
    seam1_points_y[i]=seam1_points_y_vec[i]; 
    seam1_points_z[i]=seam1_points_z_vec[i];
  }


  // setup a tf for a 'searchbox' marker so we we can see it in RVIZ - maybe someday...
  // static tf::TransformBroadcaster br_searchbox;
  // tf::Transform tf_searchbox;

  std::cout<<"Debug0"<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     Preparing Pointcloud Data                      "<<endl;
  std::cout<<"===================================================================="<<endl;

  
  // instantiate some clouds
  PointCloud::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_segment2 (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_segment3 (new pcl::PointCloud<pcl::PointXYZ>); // source cloud  // inputs to RANSAC
  PointCloud::Ptr cloud_target1 (new pcl::PointCloud<pcl::PointXYZ>);  // target cloud
  PointCloud::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZ>);  // target cloud aligned by resulting T
  //PointCloud::Ptr cloud_target3 (new pcl::PointCloud<pcl::PointXYZ>);  // target cloud final
  PointCloud::Ptr cloud_source_part1 (new pcl::PointCloud<pcl::PointXYZ>); // cylinder cloud // input to ICP
  PointCloud::Ptr cloud_source_part2 (new pcl::PointCloud<pcl::PointXYZ>); // plane cloud

  std::cout<<"Debug1"<<endl;


  // load the clouds from file (.pcd)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (source_path, *cloud_source) == -1)
  {
      std::cout<<"Couldn't read image file:"<<source_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< source_path <<std::endl<<
      cloud_source->width * cloud_source->height << " Data points from "<< source_path << std::endl;

  // load the cloud from CAD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_path, *cloud_target1) == -1)
  {
      std::cout<<"Couldn't read image file:"<<target_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< target_path <<std::endl<<
      cloud_target1->width * cloud_target1->height << " Data points from "<< target_path <<std::endl<<std::endl;

  std::cout<<"Debug2"<<endl;

  // for now each tf has three objects associated with it
  // 1) '<name>' (tf::transform)      // needed for transforms with pcl_ros
  // 2) '<name>_tf2' (tf2::transform) // not used
  // 3) '<name>_msg' (geometry_msgs)  // needed for bradcasting frames

  tf::StampedTransform *T_01 (new tf::StampedTransform);    // these are from the old 'TF'
  tf::StampedTransform *T_10 (new tf::StampedTransform);    // they are stil used for pcl_ros::transformPointCloud

  static tf2_ros::StaticTransformBroadcaster static_broadcaster; // this is the new 'TF2' way to broadcast tfs
  geometry_msgs::TransformStamped *T_01_msg (new geometry_msgs::TransformStamped);
  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";

  geometry_msgs::TransformStamped *T_10_msg (new geometry_msgs::TransformStamped);
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Processing Pointcloud Data                      "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // Filter the LiDAR cloud with a bounding box and a voxel (downsampling)
  filter_cloud(*cloud_source,*cloud_filtered, filter_box, voxel_leaf_size); 

  // Perform RANSAC Segmentation to separate clouds and find part of interest
  segment_cloud(*cloud_filtered,*cloud_source_part1,*cloud_source_part2, *cloud_segment2, *cloud_segment3, part1_type, ransac_norm_dist_wt, ransac_max_iter, ransac_dist_thrsh, ransac_k_srch, ransac_init_norm);

  // Perform ICP Cloud Registration to find location and orientation of part of interest
  register_cloud_icp(*cloud_source_part1,*cloud_target1, *T_10, *T_01, *T_10_msg, *T_01_msg, icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl,expected_results,calibration_offset);
  //register_cloud_icp(*cloud_target1,*cloud_source_part1, *T_10, *T_01, *T_10_msg, *T_01_msg, icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl,expected_results,calibration_offset);

  int N_cor=100;

  // Perform TEASER++ cloud registration
  double teaser_params[3]={1,2,3}; // temporary place holder 
  //register_cloud_teaser(*cloud_source_part1,*cloud_target1,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);
  //register_cloud_teaser(*cloud_target1,*cloud_source_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);

  //register_cloud_teaser_fpfh(*cloud_source_part1,*cloud_target1,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);
  //register_cloud_teaser_fpfh(*cloud_target1,*cloud_source_part1,*T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);

  // Align the CAD part to using the resulting transformation
  pcl_ros::transformPointCloud(*cloud_target1,*cloud_aligned,*T_01); // this works with 'pcl::PointCloud<pcl::PointXYZ>' and 'tf::Transform'
  std::cout << "Cloud aligned using resulting transformation." << std::endl;

  // optionally save segmentation results
  if(save_part1_segment){
    pcl::io::savePCDFileASCII (source_path+"_part1.pcd", *cloud_source_part1);
  }
  if(save_part2_segment){
    pcl::io::savePCDFileASCII (source_path+"_part2.pcd", *cloud_source_part2);  
  }

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Analyzing Results                                "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  analyze_results(*T_10, expected_results);
  analyze_results(*T_01, expected_results);

  T_01_msg->header.frame_id = "base_link"; T_01_msg->child_frame_id = "T_01";
  T_10_msg->header.frame_id = "base_link"; T_10_msg->child_frame_id = "T_10";

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   Pointcloud Processing Complete                   "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   Preparing Visualization                          "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  ros::Publisher pub_source = node.advertise<PointCloud> ("/cloud_source", 1) ;
  ros::Publisher pub_filtered = node.advertise<PointCloud> ("/cloud_filtered", 1) ;
  ros::Publisher pub_segment2 = node.advertise<PointCloud> ("/cloud_segment2", 1) ;
  ros::Publisher pub_segment3 = node.advertise<PointCloud> ("/cloud_segment3", 1) ;
  ros::Publisher pub_target1 = node.advertise<PointCloud> ("/cloud_target1", 1) ;
  ros::Publisher pub_aligned = node.advertise<PointCloud> ("/cloud_aligned", 1) ;
  ros::Publisher pub_part1 = node.advertise<PointCloud> ("/cloud_source_part1", 1) ;
  ros::Publisher pub_part2 = node.advertise<PointCloud> ("/cloud_source_part2", 1) ;

  cloud_source->header.frame_id = "base_link";
  cloud_filtered->header.frame_id = "base_link";
  cloud_segment2->header.frame_id = "base_link";
  cloud_segment3->header.frame_id = "base_link";
  cloud_target1->header.frame_id = "base_link";
  cloud_aligned->header.frame_id = "base_link";
  cloud_source_part1->header.frame_id = "base_link";
  cloud_source_part2->header.frame_id = "base_link";

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                        seam_detection Complete                     "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  //publish forever
  while(ros::ok())
  {

      // this is the new 'TF2' way of broadcasting tfs
      T_01_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_msg);
      T_10_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_msg);

      pub_source.publish(cloud_source);
      pub_filtered.publish(cloud_filtered);
      pub_segment2.publish(cloud_segment2);
      pub_segment3.publish(cloud_segment3);
      pub_target1.publish(cloud_target1);
      pub_aligned.publish(cloud_aligned);
      pub_part1.publish(cloud_source_part1);
      pub_part2.publish(cloud_source_part2);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}



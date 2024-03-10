/*
This source contains the several examples of point cloud registration
for testing and demonstration purposes

Tristan Hill - Weld Seam Detection - Tennessee Technological University
Robotics Research Group - Mechanical Engineering

v1.0 - 2022-12-20 

see README.md or https://github.com/thillRobot/seam_detection for documentation
*/
  
// basic file operations
#include <iostream>
#include <fstream>
#include <string>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>

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
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

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

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
//#include <teaser/point_cloud.h>
//#include <teaser/features.h>

#include <cloudutils.h>
#include <cloudregistration.h>
#include <cloudfilter.h>

//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;

bool filter_cloud_complete=0;
bool registration_complete=0;


// function calculates a difference detween the measured and expected transformation and prints the info to the console
void analyze_results(tf::Transform &tf_in,double e_results[])
{
  
  std::cout<<"Measured Rotation Matrix:"<<std::endl;  
  std::cout<<"["<<tf_in.getBasis()[0][0]<<","<<tf_in.getBasis()[0][1]<<","<<tf_in.getBasis()[0][2]<<","<<std::endl;
  std::cout     <<tf_in.getBasis()[1][0]<<","<<tf_in.getBasis()[1][1]<<","<<tf_in.getBasis()[1][2]<<","<<std::endl;
  std::cout     <<tf_in.getBasis()[2][0]<<","<<tf_in.getBasis()[2][1]<<","<<tf_in.getBasis()[2][2]<<"]"<<std::endl;

  std::cout<<"Expected,Translation: ["<<e_results[0]<<","
                                      <<e_results[1]<<","
                                      <<e_results[2]<<"]"<<std::endl;
  std::cout<<"Measured Translation: ["<<tf_in.getOrigin().getX()<<","
                                      <<tf_in.getOrigin().getY()<<","
                                      <<tf_in.getOrigin().getZ()<<"]"<<std::endl;
  std::cout<<"Difference Translation: ["<<e_results[0]-tf_in.getOrigin().getX()<<","
                                      <<e_results[1]-tf_in.getOrigin().getY()<<","
                                      <<e_results[2]-tf_in.getOrigin().getZ()<<"]"<<std::endl;

  std::cout<<"Expected Rotation: [" <<e_results[3]<<","
                                    <<e_results[4]<<","
                                    <<e_results[5]<<"]"<<std::endl;
  std::cout<<"Measured Rotation: [" <<tf_in.getRotation().getAxis().getX()
                                    <<","<<tf_in.getRotation().getAxis().getY()
                                    <<","<<tf_in.getRotation().getAxis().getZ()<<"]"<<std::endl; 
  std::cout<<"Difference Rotation: [" <<e_results[3]-tf_in.getRotation().getAxis().getX()
                                    <<","<<e_results[4]-tf_in.getRotation().getAxis().getY()
                                    <<","<<e_results[5]-tf_in.getRotation().getAxis().getZ()<<"]"<<std::endl; 

  //std::cout<<"W:"<<tf_in.getRotation().getW()<<std::endl;

}


int main(int argc, char** argv)
{

  ros::init(argc,argv,"register_clouds");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  // setup subcribers for filter_cloud_state
  //ros::Subscriber filter_cloud_state_sub = node.subscribe("/filter_cloud/filter_cloud_state", 1000, filter_cloud_stateCallback);

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    register_clouds v1.9                            "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    register_clouds: loading configuration file     "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // load ROS parameters from config file, default config matches this script name -> config/register_clouds.yaml
  bool use_icp, use_teaser, use_teaser_fpfh, save_aligned, wait_for_filter;
  node.getParam("wait_for_filter", wait_for_filter);
  node.getParam("use_icp", use_icp);
  node.getParam("use_teaser", use_teaser);
  node.getParam("use_teaser_fpfh", use_teaser_fpfh);
  node.getParam("save_aligned", save_aligned);

  // parameters that contain strings  
  std::string source_cloud_path, target_cloud_path, aligned_cloud_path, 
              source_cloud_file, target_cloud_file, aligned_cloud_file;

  node.getParam("register_clouds/source_file", source_cloud_file);
  source_cloud_path=packagepath+'/'+source_cloud_file;
  node.getParam("register_clouds/target_file", target_cloud_file);
  target_cloud_path=packagepath+'/'+target_cloud_file;\
  node.getParam("register_clouds/aligned_file", aligned_cloud_file);
  aligned_cloud_path=packagepath+'/'+aligned_cloud_file;

  // parameters that contain doubles
  double voxel_leaf_size, ransac_norm_dist_wt, ransac_max_iter, ransac_dist_thrsh, ransac_k_srch,
         icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl, icp_ran_rej_thrsh;

  // parameters that contain vectors of doubles
  std::vector<double> xs, ys, zs, filter_box_vec, 
                      ransac_init_norm_vec, expected_results_vec, calibration_offset_vec; 
                     // seam1_points_x_vec, seam1_points_y_vec, seam1_points_z_vec;
  double filter_box[6],ransac_init_norm[3],icp_params[4],
         expected_results[6],calibration_offset[6];
        // seam1_points_x[4],seam1_points_y[4],seam1_points_z[4];
  
  node.getParam("register_clouds/filter_box",  filter_box_vec);
  for(unsigned i=0; i < filter_box_vec.size(); i++)
    filter_box[i]=filter_box_vec[i]; // copy from vector to array 
  node.getParam("register_clouds/voxel_leaf_size", voxel_leaf_size);

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

  //node.getParam("expected_results",expected_results_vec);  // these four ICP parameters define the search
  //node.getParam("calibration_offset",calibration_offset_vec);  // these four ICP parameters define the search
  //for(unsigned i=0; i < expected_results_vec.size(); i++){
  //  expected_results[i]=expected_results_vec[i]; // copy into an array 
  //  calibration_offset[i]=calibration_offset_vec[i]; // copy into an array 
  // }
 
  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    register_clouds: preparing pointcloud data      "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // instantiate cloud objects
  PointCloud::Ptr source_cloud (new PointCloud);  // source cloud
  PointCloud::Ptr source_cloud_intr (new PointCloud);  // intermediate source cloud
  PointCloud::Ptr source_cloud_intr_min (new PointCloud);  // min fscore intermediate source cloud
  PointCloud::Ptr target_cloud (new PointCloud);  // target cloud
  PointCloud::Ptr corrs_cloud (new PointCloud );  // correspondence cloud   
  PointCloud::Ptr aligned_cloud_T10 (new PointCloud);  // alinged source cloud (using registration results)
  PointCloud::Ptr aligned_cloud_T01 (new PointCloud);  // alinged source cloud (using registration inverse results)
  
  // wait for pointclouds from filter_cloud
  while(!filter_cloud_complete && wait_for_filter){
    ros::spinOnce(); // update topics while waiting
    std::cout<<"waiting for filtering to complete, wait_for_filter "<<wait_for_filter<<std::endl;
  }
  std::cout<<"filtering wait complete, wait_for_filter: "<<wait_for_filter<<std::endl;
  
  bool source_loaded=0;
  bool target_loaded=0;

  std::cout<<"loading source file: "<<source_cloud_path<<std::endl;
  std::cout<<"loading target file: "<<target_cloud_path<<std::endl;
  
  while (!(source_loaded&&target_loaded)){
    // load the source cloud from PCD file, files generated with src/cad_cloud.cpp
       
    try{
      if (pcl::io::loadPCDFile<PointT> (source_cloud_path, *source_cloud) == -1)
      {
        std::cout<<"Couldn't read image file:"<<source_cloud_path;
      }else if (!source_loaded){
        std::cout << "Loaded "<<source_cloud->size()<< " data points from "<< source_cloud_file <<std::endl;
        source_loaded=1;  
      }
      // load the target cloud from PCD file
      if (pcl::io::loadPCDFile<PointT> (target_cloud_path, *target_cloud) == -1)
      {
        std::cout<<"Couldn't read image file:"<<target_cloud_path;
      }else if(!target_loaded){
        std::cout << "Loaded "<<target_cloud->size()<< " data points from "<< target_cloud_file <<std::endl;
        target_loaded=1;
      }
    }catch(...){
      std::cout<<"Could not read files"<<std::endl;
      //source_loaded=0; 
    }

  }
  
  std::cout<<"file loading loop complete"<<std::endl;    


  // downsample the clouds before registration to reduce computation
  CloudFilter filter;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
  filter.downsampleCloud(*target_cloud, *target_cloud, 0.0025); 
  filter.downsampleCloud(*source_cloud, *source_cloud, 0.0025);


  // for now each tf has three objects associated with it (more objects == more fun)
  // 1) '<name>' (tf::transform)      // needed for transforms with pcl_ros
  // 2) '<name>_tf2' (tf2::transform) // not used
  // 3) '<name>_msg' (geometry_msgs)  // needed for bradcasting frames

  tf::StampedTransform *T_01 (new tf::StampedTransform);    // these are from the old 'TF'
  tf::StampedTransform *T_10 (new tf::StampedTransform);    // they are stil used for pcl_ros::transformPointCloud
  tf::StampedTransform *T_01_min (new tf::StampedTransform);    
  tf::StampedTransform *T_10_min (new tf::StampedTransform);
  tf::StampedTransform *T_intr (new tf::StampedTransform);  // transform to intermediate starting location
  tf::StampedTransform *T_intr_inv (new tf::StampedTransform);  //inverse transform to intermediate starting location
  
  tf::StampedTransform *T_01_intr (new tf::StampedTransform);
  tf::StampedTransform *T_10_intr (new tf::StampedTransform);     
  //tf::StampedTransform *T_01_intr_min (new tf::StampedTransform);
  //tf::StampedTransform *T_10_intr_min (new tf::StampedTransform); 

  static tf2_ros::StaticTransformBroadcaster static_broadcaster; // this is the new 'TF2' way to broadcast tfs
  
  geometry_msgs::TransformStamped *T_01_msg (new geometry_msgs::TransformStamped);      // frames with parent frame base_link
  T_01_msg->header.frame_id = "T_intr_min"; T_01_msg->child_frame_id = "T_01";
  geometry_msgs::TransformStamped *T_10_msg (new geometry_msgs::TransformStamped);
  T_10_msg->header.frame_id = "T_intr_min"; T_10_msg->child_frame_id = "T_10";
  
  //geometry_msgs::TransformStamped *T_intr_msg (new geometry_msgs::TransformStamped);
  //T_intr_msg->header.frame_id = "base_link"; T_intr_msg->child_frame_id = "T_intr";
  
  geometry_msgs::TransformStamped *T_intr_min_msg (new geometry_msgs::TransformStamped);
  T_intr_min_msg->header.frame_id = "base_link"; T_intr_min_msg->child_frame_id = "T_intr_min";

  geometry_msgs::TransformStamped *T_01_intr_msg (new geometry_msgs::TransformStamped);  // frames with parent frame T_intr_min
  T_01_intr_msg->header.frame_id = "base_link"; T_01_intr_msg->child_frame_id = "T_01_intr";
  geometry_msgs::TransformStamped *T_10_intr_msg (new geometry_msgs::TransformStamped);
  T_10_intr_msg->header.frame_id = "base_link"; T_10_intr_msg->child_frame_id = "T_10_intr";

   geometry_msgs::TransformStamped *T_01_intr_min_msg (new geometry_msgs::TransformStamped);  
  T_01_intr_min_msg->header.frame_id = "base_link"; T_01_intr_min_msg->child_frame_id = "T_01_intr_min";
  geometry_msgs::TransformStamped *T_10_intr_min_msg (new geometry_msgs::TransformStamped);
  T_10_intr_min_msg->header.frame_id = "base_link"; T_10_intr_min_msg->child_frame_id = "T_10_intr_min";


  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    register_clouds: processing pointcloud data     "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  int N_cor=100;
  EigenCor cor_src_pts, cor_tgt_pts;
  Eigen::Matrix<double, 6, Eigen::Dynamic> corrs;

  double fscore; // fitness score (lower is better)
  double fscore_min=1000;

  //double alphas[1]={0}; // array of starting angles
  //int N=1;  

  double alphas[4]={0, 90, 180, 270}; // array of starting angles
  int N=4; // number of starting positions

  // set rotation and origin of a quaternion for the tf transform object
  double alpha, beta, gamma, dtr, intm;
  dtr=M_PI/180.0; // degrees to radians
  intm=0.0254;  // inches to meters

  // repeat registration for each starting value
  int i_min;
  for (int i=0;i<N;i++){

    // rotation angles for yaw pitch roll
    alpha=alphas[i]*dtr;beta=0*dtr;gamma=0*dtr; 

    // rotation matrix for Yaw Pitch Roll by alpha gamma beta
    tf::Matrix3x3 R_intr(cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma),
                         sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma),
                         -sin(beta)          , cos(beta)*sin(gamma)                                 , cos(beta)*cos(gamma));  

    // quaternion for previous rotation matrix
    tf::Quaternion q_intr;
    R_intr.getRotation(q_intr); // sets quaternion q_intr with rotation from R_intr (returns normalized quaternion?, check on this)

    T_intr->setRotation(q_intr);
    T_intr->setOrigin(tf::Vector3(0, 0, 0)); // no translation component of the transformation (is 0,0,0 default?)
    // need to normalize quaternion here?

    T_intr_inv->setData(T_intr->inverse()); // get the inverse intermediate transformation, use setData() to copy from pointer to pointer

    // transform source cloud to ith intermediate starting position 
    pcl_ros::transformPointCloud(*source_cloud, *source_cloud_intr, *T_intr);

    CloudRegistration reg;
    // perform registration starting from intermediate starting position
    
    if(use_icp){
      // Perform ICP Cloud Registration using CloudRegistration library from this package
      reg.loadConfig(reg.getConfig()); // use values in default config, this is goofy fix this     
      std::cout<<"CloudRegistration config: "<<reg.getConfig()<<std::endl;
      std::cout<<"CloudRegistration icp_max_corr_dist: "<<reg.icp_max_corr_dist<<std::endl;
      fscore=reg.registerCloudICP(*source_cloud_intr, *target_cloud, *T_10_intr, *T_01_intr, *T_10_intr_msg, *T_01_intr_msg);
      std::cout << "ICP completed with fitness score: " << fscore << std::endl;
    
    }else if (use_teaser){
      // Perform TEASER++ cloud registration
      double teaser_params[3]={1,2,3}; // temporary place holder 
      reg.registerCloudTeaser(*source_cloud_intr,*target_cloud,  *T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params);
    
    }else if(use_teaser_fpfh){
      // Perform TEASER++ cloud registration with Fast Point Feature Histograms (FPFH) descriptors  
      double teaser_params[3]={1,2,3}; // temporary place holder 
      teaser::FPFHEstimation features;   
      corrs=reg.registerCloudTeaserFPFH(*source_cloud_intr, *target_cloud, *corrs_cloud, 
                                        *T_10, *T_01, *T_10_msg, *T_01_msg, teaser_params, features);
      std::cout<<"registerCloudTeaserFPFH() correspondences"<<std::endl;
      std::cout<<"size: "<<corrs.size()<<std::endl;
    }
    
    // backout intermediate starting point transformation here, revisit this soon, or let sz do it...

    // find intermediate starting position which gives lowest registration score
    if (fscore<fscore_min){
      fscore_min=fscore;
      //T_intr_min=T_intr;   // dont copy the pointers directly, this will break the minimization routine
      //T_01_intr_min=T_01_intr;
      //T_10_intr_min=T_10_intr;
      i_min=i;

      // align the source cloud using the resulting transformation only if fscore has improved
      pcl_ros::transformPointCloud(*source_cloud_intr, *aligned_cloud_T01, *T_01_intr);
      pcl_ros::transformPointCloud(*source_cloud_intr, *aligned_cloud_T10, *T_10_intr); // this works with 'pcl::PointCloud<pcl::PointXYZ>' and 'tf::Transform'
      pcl_ros::transformPointCloud(*source_cloud, *source_cloud_intr_min, *T_intr);

      // align weld seam points using transformation

      // copy the tranform from pointer      
      tf::StampedTransform T_intr_tmp(*T_intr);
      tf::StampedTransform T_01_intr_tmp(*T_01_intr);

      tf::Vector3 P0_target, P1_target, P2_target,
                  P0_source, P1_source, P2_source,
                  P0_source_inches, P1_source_inches, P2_source_inches;

      // points for shape3
      //tf::Vector3 P0_target_inches(0, 0, 0-6);   // weld points in inches
      //tf::Vector3 P1_target_inches(0, 2, 4.5-6);
      //tf::Vector3 P2_target_inches(12.5, 2, 4.5-6);

      // points for shape2
      tf::Vector3 P0_target_inches(0, 0, 0-9);   // weld points in inches
      tf::Vector3 P1_target_inches(0, 0, 2-9);
      tf::Vector3 P2_target_inches(14, 0, 2-9);
      
      P0_target=P0_target_inches*intm;       // convert to meters 
      P1_target=P1_target_inches*intm;
      P2_target=P2_target_inches*intm;

      P0_source=T_intr_tmp.inverse()*T_01_intr_tmp*P0_target;
      P1_source=T_intr_tmp.inverse()*T_01_intr_tmp*P1_target;
      P2_source=T_intr_tmp.inverse()*T_01_intr_tmp*P2_target;

      P0_source_inches=P0_source/intm;       // convert to inches
      P1_source_inches=P1_source/intm;
      P2_source_inches=P2_source/intm;

      std::cout<<"P0_target: ["<<P0_target.x()<<","<<P0_target.y()<<","<<P0_target.z()<<"]"<<std::endl;
      std::cout<<"P0_source: ["<<P0_source.x()<<","<<P0_source.y()<<","<<P0_source.z()<<"]"<<std::endl;
      std::cout<<"P1_target: ["<<P1_target.x()<<","<<P1_target.y()<<","<<P1_target.z()<<"]"<<std::endl;
      std::cout<<"P1_source: ["<<P1_source.x()<<","<<P1_source.y()<<","<<P1_source.z()<<"]"<<std::endl;
      std::cout<<"P2_target: ["<<P2_target.x()<<","<<P2_target.y()<<","<<P2_target.z()<<"]"<<std::endl;
      std::cout<<"P2_source: ["<<P2_source.x()<<","<<P2_source.y()<<","<<P2_source.z()<<"]"<<std::endl;

      std::cout<<"P0_target_inches: ["<<P0_target_inches.x()<<","<<P0_target_inches.y()<<","<<P0_target_inches.z()<<"]"<<std::endl;
      std::cout<<"P0_source_inches: ["<<P0_source_inches.x()<<","<<P0_source_inches.y()<<","<<P0_source_inches.z()<<"]"<<std::endl;
      std::cout<<"P1_target_inches: ["<<P1_target_inches.x()<<","<<P1_target_inches.y()<<","<<P1_target_inches.z()<<"]"<<std::endl;
      std::cout<<"P1_source_inches: ["<<P1_source_inches.x()<<","<<P1_source_inches.y()<<","<<P1_source_inches.z()<<"]"<<std::endl;
      std::cout<<"P2_target_inches: ["<<P2_target_inches.x()<<","<<P2_target_inches.y()<<","<<P2_target_inches.z()<<"]"<<std::endl;
      std::cout<<"P2_source_inches: ["<<P2_source_inches.x()<<","<<P2_source_inches.y()<<","<<P2_source_inches.z()<<"]"<<std::endl;

      // write the points to a gcode file called 'move_P1_P2', the source points will be used
      
      ofstream outfile;
      outfile.open (packagepath+"/"+"/gcode/move_P1_P2.txt");
      outfile <<"G1 X"<<P1_source_inches.x()<<" Y"<<P1_source_inches.y()<<" Z"<<P1_source_inches.z()
              <<" A60 B10 C175 F200"<<std::endl;
      outfile <<"G4 P0.2"<<std::endl;        
      outfile << "G1 X"<<P2_source_inches.x()<<" Y"<<P2_source_inches.y()<<" Z"<<P2_source_inches.z()
              <<" A60 B10 C175 F150"<<std::endl;
      outfile.close();

      // update the messages to be published after updating transforms upon finding minimum
      //tf::transformStampedTFToMsg(*T_intr, *T_intr_msg);
      tf::transformStampedTFToMsg(*T_intr_inv, *T_intr_min_msg);
      tf::transformStampedTFToMsg(*T_01_intr, *T_01_intr_min_msg);
      tf::transformStampedTFToMsg(*T_10_intr, *T_10_intr_min_msg);

      tf::transformStampedTFToMsg(*T_01_intr, *T_01_msg);
      tf::transformStampedTFToMsg(*T_10_intr, *T_10_msg);

      std::cout << "Score improved from starting position "<< i << ", recording registration results" << std::endl;
    }else{
      std::cout << "Score not improved from starting position "<< i << ", skipping" << std::endl;
    }
  }

  // update the messages to be published after updating transforms
  //tf::transformStampedTFToMsg(*T_intr, *T_intr_msg);
  //tf::transformStampedTFToMsg(*T_intr_min, *T_intr_min_msg);
  //tf::transformStampedTFToMsg(*T_01_intr_min, *T_01_intr_min_msg);
  //tf::transformStampedTFToMsg(*T_10_intr_min, *T_10_intr_min_msg);
  //tf::transformStampedTFToMsg(*T_01_intr, *T_01_intr_msg);
  //tf::transformStampedTFToMsg(*T_10_intr, *T_10_intr_msg);

  std::cout << "Cloud aligned from starting position "<< i_min << " using best registration results" << std::endl;
   
  // set relative frame references (this seems like it is repeated, check on this)
  
  T_01_msg->header.frame_id = "T_intr_min"; T_01_msg->child_frame_id = "T_01"; // frames with parent frame base_link
  T_10_msg->header.frame_id = "T_intr_min"; T_10_msg->child_frame_id = "T_10";
  
  //T_01_min_msg->header.frame_id = "base_link"; T_01_min_msg->child_frame_id = "T_01_min"; // frames with parent frame base_link
  //T_10_min_msg->header.frame_id = "base_link"; T_10_min_msg->child_frame_id = "T_10_min";

  //T_intr_msg->header.frame_id = "base_link"; T_intr_msg->child_frame_id = "T_intr";
  T_intr_min_msg->header.frame_id = "base_link"; T_intr_min_msg->child_frame_id = "T_intr_min"; 

  //T_01_intr_msg->header.frame_id = "T_intr_min"; T_01_intr_msg->child_frame_id = "T_01_intr"; // frames with parent frame T_intr
  //T_10_intr_msg->header.frame_id = "T_intr_min"; T_10_intr_msg->child_frame_id = "T_10_intr";

  T_01_intr_min_msg->header.frame_id = "base_link"; T_01_intr_min_msg->child_frame_id = "T_01_intr_min";
  T_10_intr_min_msg->header.frame_id = "base_link"; T_10_intr_min_msg->child_frame_id = "T_10_intr_min";


  
  // save aligned cloud in PCD file (alignment still needs some work, revisit next!)
  if(save_aligned){
    std::cout<<"Writing aligned cloud to:"<< aligned_cloud_path <<std::endl;
    //pcl::io::savePCDFileASCII (aligned_cloud_path, *aligned_cloud_T01); // this one should be used, check this
    pcl::io::savePCDFileASCII (aligned_cloud_path, *aligned_cloud_T10);
    std::cout<<"Aligned cloud written to:"<< aligned_cloud_path <<std::endl;
  }

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    register_clouds: analyzing results              "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  analyze_results(*T_10, expected_results);
  analyze_results(*T_01, expected_results);

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    register_clouds: preparing visualization        "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  ros::Publisher source_pub = node.advertise<PointCloud> ("/source_cloud", 1);
  ros::Publisher source_intr_min_pub = node.advertise<PointCloud> ("/source_cloud_intr_min", 1);
  ros::Publisher target_pub = node.advertise<PointCloud> ("/target_cloud", 1);

  ros::Publisher aligned_T01_pub = node.advertise<PointCloud> ("/aligned_cloud_T01", 1);
  ros::Publisher aligned_T10_pub = node.advertise<PointCloud> ("/aligned_cloud_T10", 1);
  
  source_cloud->header.frame_id = "base_link";
  source_cloud_intr_min->header.frame_id = "base_link";
  target_cloud->header.frame_id = "base_link";
  aligned_cloud_T01->header.frame_id = "base_link"; // should be base link or T_intr?
  aligned_cloud_T10->header.frame_id = "base_link";
  
  ros::Publisher source_markers_pub = node.advertise<visualization_msgs::MarkerArray>( "source_markers", 0 );
  visualization_msgs::MarkerArray source_markers;
  ros::Publisher target_markers_pub = node.advertise<visualization_msgs::MarkerArray>( "target_markers", 0 );
  visualization_msgs::MarkerArray target_markers;

  visualization_msgs::Marker source_marker, target_marker;
  source_marker.header.frame_id = "base_link";
  source_marker.header.stamp = ros::Time();
  //marker.ns = "my_namespace";
  source_marker.type = visualization_msgs::Marker::SPHERE;
  source_marker.action = visualization_msgs::Marker::ADD;
  source_marker.pose.orientation.x = 0.0;
  source_marker.pose.orientation.y = 0.0;
  source_marker.pose.orientation.z = 0.0;
  source_marker.pose.orientation.w = 1.0;
  source_marker.scale.x = 0.005;
  source_marker.scale.y = 0.005;
  source_marker.scale.z = 0.005;
  source_marker.color.a = 1.0; // Don't forget to set the alpha!
  source_marker.color.r = 255.0/255.0;
  source_marker.color.g = 255.0/255.0;
  source_marker.color.b = 255.0/255.0;
  
  target_marker.header.frame_id = "base_link";
  target_marker.header.stamp = ros::Time();
  target_marker.type = visualization_msgs::Marker::SPHERE;
  target_marker.action = visualization_msgs::Marker::ADD;
  target_marker.pose.orientation.x = 0.0;
  target_marker.pose.orientation.y = 0.0;
  target_marker.pose.orientation.z = 0.0;
  target_marker.pose.orientation.w = 1.0;
  target_marker.scale.x = 0.005;
  target_marker.scale.y = 0.005;
  target_marker.scale.z = 0.005;
  target_marker.color.a = 1.0; // Don't forget to set the alpha!
  target_marker.color.r = 255.0/255.0;
  target_marker.color.g = 16.0/255.0;
  target_marker.color.b = 240.0/255.0;
  
  if(use_teaser_fpfh){
    for(size_t i = 0; i < corrs.cols(); i++){  
      source_marker.id = i;
      source_marker.pose.position.x = corrs(0,i);
      source_marker.pose.position.y = corrs(1,i);
      source_marker.pose.position.z = corrs(2,i);

      target_marker.id = i;
      target_marker.pose.position.x = corrs(3,i);
      target_marker.pose.position.y = corrs(4,i);
      target_marker.pose.position.z = corrs(5,i);
      //cout << corrs[i].first << ", " << corrs[i].second << endl;
      //std::cout<<"i"<<std::endl;
      source_markers.markers.push_back(source_marker); // add the marker to the marker array
      target_markers.markers.push_back(target_marker); // add the marker to the marker array   
    }
  }

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    register_clouds: register clouds complete       "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  //publish forever
  while(ros::ok())
  {
      // this is the new 'TF2' way of broadcasting tfs
      T_01_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_msg);
      T_10_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_msg);
      
      //T_01_min_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_min_msg);
      //T_10_min_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_min_msg);
      
      //T_intr_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_intr_msg);
      T_intr_min_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_intr_min_msg);

      //T_01_intr_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_intr_msg);
      //T_10_intr_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_intr_msg);

      T_01_intr_min_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_01_intr_min_msg);
      T_10_intr_min_msg->header.stamp = ros::Time::now(); static_broadcaster.sendTransform(*T_10_intr_min_msg);
      
      source_pub.publish(source_cloud);
      source_intr_min_pub.publish(source_cloud_intr_min);
      
      target_pub.publish(target_cloud);
      aligned_T01_pub.publish(aligned_cloud_T01);
      aligned_T10_pub.publish(aligned_cloud_T10);

      source_markers_pub.publish(source_markers);
      target_markers_pub.publish(target_markers);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

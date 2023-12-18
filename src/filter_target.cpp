/*
// This program applies a series of filters and processes to the input pointcloud
// to prepare for registration 
// A .pcd file is read as input and a new .pcd file is written as output
// A target .pcd file can also be input for cluster selection
// Tristan Hill - 07/14/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation

// this code is based on PCL example code: 
// https://pcl.readthedocs.io/en/latest/passthrough.html
// https://pcl.readthedocs.io/en/latest/cluster_extraction.html

// PCA box example code algorithm adapted from:
// http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
*/

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include "ros/package.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;


bool get_target_complete=0;
bool filtered_target_saved=0; 

void get_target_state_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data){
    ROS_INFO("get_target in progress, waiting to begin filtering ...");
  }
  else if (msg->data&&!get_target_complete){
    ROS_INFO("get_target complete, beginning filtering");
    get_target_complete=1;
  }
}

// this function applies a bounding box and a voxel filter to the input cloud
void filter_cloud(PointCloud &input, PointCloud &output, double box[], double leaf_size, bool translate_output, bool auto_bounds)
{

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(input,*cloud);

  std::cout<<"Beginning filter_cloud() function" << std::endl;
  std::cout<<"Before filtering there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
  
  double box_length, box_width, box_height;
  box_length=0.25;
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

    std::cout<<"Using automatic bounding box limits: "<<box[0]<<","<<box[1]<<","<<box[2]<<","<<box[3]<<","<<box[4]<<","<<box[5]<<"]"<< std::endl;
  }else{
    std::cout<<"Using bounding box limits: "<<box[0]<<","<<box[1]<<","<<box[2]<<","<<box[3]<<","<<box[4]<<","<<box[5]<<"] from config file"<< std::endl;
  }

  //Apply Bounding Box Filter
  pcl::PassThrough<pcl::PointXYZ> pass; //cloud_input
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

  // Apply Voxel Filter 

  if (leaf_size>0)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (leaf_size, leaf_size, leaf_size); // use "001f","001f","0001f" or "none" to set voxel leaf size
    vox.filter (*cloud);
    std::cout<<"After voxel filtering there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
  }else
  {
    std::cout<<"No voxel filtering"<< std::endl;
  }

  // translate data by location of corner of bounding box
  /*  METHOD #2: Using a Affine3f
     This method is easier and less error prone
   */
   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
 
   // Define a translation of 2.5 meters on the x axis.
   transform_2.translation() << -box[0], -box[2], -box[4];

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);

  if (translate_output){
    pcl::copyPointCloud(*transformed_cloud, output);
  }else{
    pcl::copyPointCloud(*cloud,output);
  }

}

// this function performs Euclidean cluster extraction to separate parts of the pointcloud 
void cluster_cloud(PointCloud &input, PointCloud &output0, PointCloud &output1, PointCloud &output2, PointCloud &output3, PointCloud &output4 ){

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(input,*cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
 
  int j = 0;
  for (const auto& cluster : cluster_indices) 
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    } //*
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if (j==0){ // save the first five clusters for now, this could be improved with iteration
      pcl::copyPointCloud(*cloud_cluster,output0);
    }else if(j==1){
      pcl::copyPointCloud(*cloud_cluster,output1);
    }else if(j==2){
      pcl::copyPointCloud(*cloud_cluster,output2);
    }else if(j==3){
      pcl::copyPointCloud(*cloud_cluster,output3);
    }else if(j==4){
      pcl::copyPointCloud(*cloud_cluster,output4);
    }

    std::cout << "PointCloud representing cluster"<<j<<" has "<< cloud_cluster->size() << " data points." << std::endl;
    j++;
  }

}

// this function finds the minimum oriented bounding box of a cloud using principle component analysis
void pcabox_cloud(PointCloud &input, Eigen::Quaternionf& bbox_quaternion, Eigen::Vector3f& bbox_transform, Eigen::Vector3f& bbox_dimensions){

  PointCloud::Ptr cloud (new PointCloud); //make working copy of the input cloud
  pcl::copyPointCloud(input,*cloud);

  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloud, pcaCentroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
  
  bbox_dimensions[0]=maxPoint.x-minPoint.x;
  bbox_dimensions[1]=maxPoint.y-minPoint.y;
  bbox_dimensions[2]=maxPoint.z-minPoint.z;

  double bbox_volume, bbox_aspect_ratio;
  bbox_volume=bbox_dimensions[0]*bbox_dimensions[1]*bbox_dimensions[2]; // calculate volume as product of dimensions
  bbox_aspect_ratio=bbox_dimensions.maxCoeff()/bbox_dimensions.minCoeff(); // calculate aspect ratio as max dimension / min dimension

  std::cout<<"volume: "<<bbox_volume<<std::endl;
  std::cout<<"aspect ratio: "<<bbox_aspect_ratio<<std::endl;

  bbox_quaternion=bboxQuaternion;
  bbox_transform=bboxTransform;

  return;

}

// this function finds the minimum oriented bounding box of a cloud using principle component analysis
double score_cluster(PointCloud &input, PointCloud &target){

  PointCloud::Ptr input_cloud (new PointCloud); //make working copy of the input cloud
  pcl::copyPointCloud(input,*input_cloud);
  PointCloud::Ptr target_cloud (new PointCloud); //make working copy of the target cloud
  pcl::copyPointCloud(target,*target_cloud);

  // Compute principal directions
  Eigen::Vector4f inputCentroid, targetCentroid;
  pcl::compute3DCentroid(*input_cloud, inputCentroid);
  pcl::compute3DCentroid(*target_cloud, targetCentroid);
  Eigen::Matrix3f input_covariance, target_covariance;
  computeCovarianceMatrixNormalized(*input_cloud, inputCentroid, input_covariance);
  computeCovarianceMatrixNormalized(*target_cloud, targetCentroid, target_covariance);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> input_eigen_solver(input_covariance, Eigen::ComputeEigenvectors);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> target_eigen_solver(target_covariance, Eigen::ComputeEigenvectors);

  Eigen::Matrix3f input_eigenVectorsPCA = input_eigen_solver.eigenvectors();
  Eigen::Matrix3f target_eigenVectorsPCA = target_eigen_solver.eigenvectors();

  input_eigenVectorsPCA.col(2) = input_eigenVectorsPCA.col(0).cross(input_eigenVectorsPCA.col(1));
  target_eigenVectorsPCA.col(2) = target_eigenVectorsPCA.col(0).cross(target_eigenVectorsPCA.col(1));

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f input_projectionTransform(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f target_projectionTransform(Eigen::Matrix4f::Identity());

  input_projectionTransform.block<3,3>(0,0) = input_eigenVectorsPCA.transpose();
  input_projectionTransform.block<3,1>(0,3) = -1.f * (input_projectionTransform.block<3,3>(0,0) * inputCentroid.head<3>());
  
  target_projectionTransform.block<3,3>(0,0) = target_eigenVectorsPCA.transpose();
  target_projectionTransform.block<3,1>(0,3) = -1.f * (target_projectionTransform.block<3,3>(0,0) * targetCentroid.head<3>());
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud(*input_cloud, *inputPointsProjected, input_projectionTransform);
  pcl::transformPointCloud(*target_cloud, *targetPointsProjected, target_projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.

  pcl::PointXYZ input_minPoint, input_maxPoint, target_minPoint, target_maxPoint;
  pcl::getMinMax3D(*inputPointsProjected, input_minPoint, input_maxPoint);
  pcl::getMinMax3D(*targetPointsProjected, target_minPoint, target_maxPoint);
  const Eigen::Vector3f input_meanDiagonal = 0.5f*(input_maxPoint.getVector3fMap() + input_minPoint.getVector3fMap());
  const Eigen::Vector3f target_meanDiagonal = 0.5f*(target_maxPoint.getVector3fMap() + target_minPoint.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf inputQuaternion(input_eigenVectorsPCA); 
  const Eigen::Quaternionf targetQuaternion(target_eigenVectorsPCA); 
  
  const Eigen::Vector3f inputTransform = input_eigenVectorsPCA * input_meanDiagonal + inputCentroid.head<3>();
  const Eigen::Vector3f targetTransform = target_eigenVectorsPCA * target_meanDiagonal + targetCentroid.head<3>();
  
  Eigen::Vector3f input_dimensions, target_dimensions;
  input_dimensions[0]=input_maxPoint.x-input_minPoint.x;
  input_dimensions[1]=input_maxPoint.y-input_minPoint.y;
  input_dimensions[2]=input_maxPoint.z-input_minPoint.z;

  target_dimensions[0]=target_maxPoint.x-target_minPoint.x;
  target_dimensions[1]=target_maxPoint.y-target_minPoint.y;
  target_dimensions[2]=target_maxPoint.z-target_minPoint.z;

  double input_volume, input_aspect_ratio, target_volume, target_aspect_ratio, score;
  input_volume=input_dimensions[0]*input_dimensions[1]*input_dimensions[2]; // calculate volume as product of dimensions
  input_aspect_ratio=input_dimensions.maxCoeff()/input_dimensions.minCoeff(); // calculate aspect ratio as max dimension / min dimension
  target_volume=target_dimensions[0]*target_dimensions[1]*target_dimensions[2]; // calculate volume as product of dimensions
  target_aspect_ratio=target_dimensions.maxCoeff()/target_dimensions.minCoeff(); // calculate aspect ratio as max dimension / min dimension

  //score=pow((pow((input_volume-target_volume),2)*pow((input_aspect_ratio-target_aspect_ratio),2)) , 0.5) ;
  // volume diff needs large weight to offset order of magnitude difference in units btwn volume and aspect ratio (1000:1 -> equal weight) 

  // weights successfully tested shape1.pcd vs aubo_cloud_out_00-09.pcd
  score=10000*abs(input_volume-target_volume)
        +1.0*abs(input_aspect_ratio-target_aspect_ratio)
        +0.0*abs(input_dimensions.maxCoeff()-target_dimensions.maxCoeff());
  
  // weights successfully tested shape2.pcd vs aubo_cloud_out_09-15.pcd
  //score= 100000*abs(input_volume-target_volume)
  //      +1.0*abs(input_aspect_ratio-target_aspect_ratio)
  //      +1000.0*abs(input_dimensions.maxCoeff()-target_dimensions.maxCoeff());

  std::cout<<"input volume: "<<input_volume<<std::endl;
  std::cout<<"input aspect ratio: "<<input_aspect_ratio<<std::endl;
  std::cout<<"target volume: "<<target_volume<<std::endl;
  std::cout<<"target aspect ratio: "<<target_aspect_ratio<<std::endl;
  std::cout<<"comparison score: "<<score<<std::endl<<std::endl;
  
  return score;

}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"filter_target");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);
  
  // setup subcribers for get_target_state
  ros::Subscriber get_target_state_sub = node.subscribe("/get_target/get_target_state", 1000, get_target_state_callback);

  // publisher for filter_target_state
  ros::Publisher filter_target_state_pub = node.advertise<std_msgs::Bool> ("/filter_target/filter_target_state", 1);
  
  std_msgs::Bool filter_target_state_msg;
  filter_target_state_msg.data=filtered_target_saved;


  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_target v1.6                              "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_target: loading configuration file       "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // this is not true, the params are loaded in the launch file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters 
  bool save_output, translate_output, automatic_bounds, use_clustering, new_scan;
  node.getParam("save_output", save_output);
  node.getParam("translate_output", translate_output);
  node.getParam("automatic_bounds", automatic_bounds);
  node.getParam("use_clustering", use_clustering);

  // parameters that contain strings  
  std::string input_path, output_path, target_path, input_file, output_file, target_file; 
  node.getParam("filter_target/input_file", input_file);
  input_path=packagepath+'/'+input_file;
  node.getParam("filter_target/output_file", output_file);
  output_path=packagepath+'/'+output_file;
  node.getParam("filter_target/target_file", target_file);
  target_path=packagepath+'/'+target_file;

  // parameters that contain doubles
  double voxel_leaf_size;

  // parameters that contain vectors of doubles
  std::vector<double> filter_box_vec;
  double filter_box[6];
  
  node.getParam("filter_target/filter_box",  filter_box_vec);
  for(unsigned i=0; i < filter_box_vec.size(); i++)
    filter_box[i]=filter_box_vec[i]; // copy from vector to array 
  node.getParam("filter_target/voxel_leaf_size", voxel_leaf_size);

  // get additional parameters set in launch file
  bool saving_source=0;
  bool saving_target=0;
  

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_target: preparing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  
  // instantiate some cloud pointers
  PointCloud::Ptr target_input (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr target_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr target_filtered_target (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr target_cluster0 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr target_cluster1 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr target_cluster2 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr target_cluster3 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr target_cluster4 (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr target_cluster_min (new pcl::PointCloud<pcl::PointXYZ>);  // cluster with the min score (best)

  // wait for pointcloud from get_target
  while(!get_target_complete){
    ros::spinOnce(); // update topics while waiting
  }
  
  std::cout << "Loading cloud files:" <<std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, *target_input) == -1)
  {
      std::cout<<"Couldn't read cloud_input file:"<<input_path;
      return (-1);
  }
  std::cout << "Loaded input cloud file: "<< input_path <<std::endl<<
    target_input->width * target_input->height << " Data points from "<< input_path << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_path, *target_filtered_target) == -1)
  {
      std::cout<<"Couldn't read cloud_target file:"<<target_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< target_path <<std::endl<<
    target_filtered_target->width * target_filtered_target->height << " Data points from "<< target_path << std::endl;  

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                    filter_target: processing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // Filter the LiDAR cloud with a bounding box and a voxel (downsampling)
  filter_cloud(*target_input,*target_filtered, filter_box, voxel_leaf_size, translate_output, automatic_bounds); 
  
  Eigen::Quaternionf target_quaternion, cluster0_quaternion, cluster1_quaternion, cluster2_quaternion, cluster3_quaternion, cluster4_quaternion, marker_quaternion;
  Eigen::Vector3f target_translation, cluster0_translation, cluster1_translation, cluster2_translation, cluster3_translation, cluster4_translation, marker_translation;
  Eigen::Vector3f target_dimensions, cluster0_dimensions, cluster1_dimensions, cluster2_dimensions, cluster3_dimensions, cluster4_dimensions, marker_dimensions;
  
  //double target_volume, cluster0_volume, cluster1_volume, cluster2_volume, cluster3_volume, cluster4_volume;
  //double target_aspect_ratio, cluster0_aspect_ratio, cluster1_aspect_ratio, cluster2_aspect_ratio, cluster3_aspect_ratio, cluster4_aspect_ratio;

  if(use_clustering){

    // find clusters in filtered cloud (five clusters hardcoded is messy, fix this somehow)
    cluster_cloud(*target_filtered, *target_cluster0, *target_cluster1, *target_cluster2, *target_cluster3, *target_cluster4);

    // find the minimum bounding box for the target cluster
    pcabox_cloud(*target_filtered_target, target_quaternion, target_translation, target_dimensions); 

    // find the minimum bounding box for a five clusters
    pcabox_cloud(*target_cluster0, cluster0_quaternion, cluster0_translation, cluster0_dimensions);
    pcabox_cloud(*target_cluster1, cluster1_quaternion, cluster1_translation, cluster1_dimensions);
    pcabox_cloud(*target_cluster2, cluster2_quaternion, cluster2_translation, cluster2_dimensions);
    pcabox_cloud(*target_cluster3, cluster3_quaternion, cluster3_translation, cluster3_dimensions);
    pcabox_cloud(*target_cluster4, cluster4_quaternion, cluster4_translation, cluster4_dimensions);

    // get score for each cluster
    double score0, score1, score2, score3, score4, score;
    //score=100;
    score0=score_cluster(*target_cluster0, *target_filtered_target);
    score1=score_cluster(*target_cluster1, *target_filtered_target);
    score2=score_cluster(*target_cluster2, *target_filtered_target);
    score3=score_cluster(*target_cluster3, *target_filtered_target);
    score4=score_cluster(*target_cluster4, *target_filtered_target);

    score=score0; // assume first score is the lowest, always save
    //if (score0<score){
     score=score0;
     pcl::copyPointCloud(*target_cluster0, *target_cluster_min);
     pcl::io::savePCDFileASCII (output_path, *target_cluster0);
     std::cout<<"cluster0 written to:"<< output_path <<std::endl;
    //}
    
    if (score1<score){
      score=score1;
      pcl::copyPointCloud(*target_cluster1, *target_cluster_min);
      pcl::io::savePCDFileASCII (output_path, *target_cluster1);
      std::cout<<"cluster1 cloud written to:"<< output_path <<std::endl;
    }
    if (score2<score){
      score=score2;
      pcl::copyPointCloud(*target_cluster2, *target_cluster_min);
      pcl::io::savePCDFileASCII (output_path, *target_cluster2);
      std::cout<<"cluster2 cloud written to:"<< output_path <<std::endl;
    }
    if (score3<score){
      score=score3;
      pcl::copyPointCloud(*target_cluster3, *target_cluster_min);
      pcl::io::savePCDFileASCII (output_path, *target_cluster3);
      std::cout<<"cluster3 cloud written to:"<< output_path <<std::endl;
    }
    if (score4<score){
      score=score4;
      pcl::copyPointCloud(*target_cluster4, *target_cluster_min);
      pcl::io::savePCDFileASCII (output_path, *target_cluster4);
      std::cout<<"cluster4 cloud written to:"<< output_path <<std::endl;
    }
    std::cout<<"the lowest score is:"<<score<<std::endl;
  } 

  // save filtered cloud 
  if(save_output&&!use_clustering){
    pcl::io::savePCDFileASCII (output_path, *target_filtered);
    std::cout<<"Filtered cloud written to:"<< output_path <<std::endl;
  }
  filtered_target_saved=1; // this should be moved?


  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                   filter_target: pointcloud processing complete     "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                   filter_target: preparing visualization            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  ros::Publisher pub_input = node.advertise<PointCloud> ("/target_input", 1) ;
  ros::Publisher pub_filtered = node.advertise<PointCloud> ("/target_filtered", 1) ;
  ros::Publisher pub_filtered_target = node.advertise<PointCloud> ("/target_filtered_target", 1) ;
  ros::Publisher pub_cluster0 = node.advertise<PointCloud> ("/target_cluster0", 1) ;
  ros::Publisher pub_cluster1 = node.advertise<PointCloud> ("/target_cluster1", 1) ;
  //ros::Publisher pub_cluster2 = node.advertise<PointCloud> ("/target_cluster2", 1) ;
  //ros::Publisher pub_cluster3 = node.advertise<PointCloud> ("/target_cluster3", 1) ;
  //ros::Publisher pub_cluster4 = node.advertise<PointCloud> ("/target_cluster4", 1) ;
  ros::Publisher pub_cluster_min = node.advertise<PointCloud> ("/target_cluster_min", 1) ;

  target_input->header.frame_id = "base_link";
  target_filtered->header.frame_id = "base_link";
  target_filtered_target->header.frame_id = "base_link";
  target_cluster0->header.frame_id = "base_link";
  target_cluster1->header.frame_id = "base_link";
  //target_cluster2->header.frame_id = "base_link";
  //target_cluster3->header.frame_id = "base_link";
  //target_cluster4->header.frame_id = "base_link";
  target_cluster_min->header.frame_id = "base_link";

  ros::Publisher target_marker_pub = node.advertise<visualization_msgs::Marker>( "target_marker", 0 );
  ros::Publisher cluster_markers_pub = node.advertise<visualization_msgs::MarkerArray>( "target_cluster_markers", 0 );
  visualization_msgs::MarkerArray cluster_markers;
  visualization_msgs::Marker target_marker, cluster_marker;
  if (use_clustering){
    
    // single marker cube for target cloud pcabox
    target_marker.header.frame_id = "base_link";
    target_marker.header.stamp = ros::Time();
    target_marker.type = visualization_msgs::Marker::CUBE;
    target_marker.action = visualization_msgs::Marker::ADD;

    target_marker.color.a = 0.15; 
    target_marker.color.r = 240.0/255.0;
    target_marker.color.g = 240.0/255.0;
    target_marker.color.b = 240.0/255.0;

    target_marker.id = 999;

    target_marker.pose.orientation.x = target_quaternion.vec()[0];
    target_marker.pose.orientation.y = target_quaternion.vec()[1];
    target_marker.pose.orientation.z = target_quaternion.vec()[2];
    target_marker.pose.orientation.w = target_quaternion.w();

    target_marker.scale.x = target_dimensions[0];
    target_marker.scale.y = target_dimensions[1];
    target_marker.scale.z = target_dimensions[2];

    target_marker.pose.position.x = target_translation[0];
    target_marker.pose.position.y = target_translation[1];
    target_marker.pose.position.z = target_translation[2];

    // array of markers for pointcloud clusters 
    cluster_marker.header.frame_id = "base_link";
    cluster_marker.header.stamp = ros::Time();
    cluster_marker.type = visualization_msgs::Marker::CUBE;
    cluster_marker.action = visualization_msgs::Marker::ADD;

    cluster_marker.color.a = 0.15; // Don't forget to set the alpha!
    cluster_marker.color.r = 255.0/255.0;
    cluster_marker.color.g = 0.0/255.0;
    cluster_marker.color.b = 255.0/255.0;
    
    for(size_t i = 0; i < 5; i++){  

      if (i==0){  // select which cluster result to copy pcabox objects from
        marker_quaternion=cluster0_quaternion;
        marker_translation=cluster0_translation;
        marker_dimensions=cluster0_dimensions;
      }else if(i==1){
        marker_quaternion=cluster1_quaternion;
        marker_translation=cluster1_translation;
        marker_dimensions=cluster1_dimensions;
      }/*else if(i==2){
        marker_quaternion=cluster2_quaternion;
        marker_translation=cluster2_translation;
        marker_dimensions=cluster2_dimensions;
      }else if(i==3){
        marker_quaternion=cluster3_quaternion;
        marker_translation=cluster3_translation;
        marker_dimensions=cluster3_dimensions;
      }else if(i==4){
        marker_quaternion=cluster4_quaternion;
        marker_translation=cluster4_translation;
        marker_dimensions=cluster4_dimensions;
      }*/

      cluster_marker.id = i;

      cluster_marker.pose.orientation.x = marker_quaternion.vec()[0];
      cluster_marker.pose.orientation.y = marker_quaternion.vec()[1];
      cluster_marker.pose.orientation.z = marker_quaternion.vec()[2];
      cluster_marker.pose.orientation.w = marker_quaternion.w();

      cluster_marker.scale.x = marker_dimensions[0];
      cluster_marker.scale.y = marker_dimensions[1];
      cluster_marker.scale.z = marker_dimensions[2];

      cluster_marker.pose.position.x = marker_translation[0];
      cluster_marker.pose.position.y = marker_translation[1];
      cluster_marker.pose.position.z = marker_translation[2];

      cluster_markers.markers.push_back(cluster_marker); // add the marker to the marker array   
    }

  }
    
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                        filter_target: complete                     "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  //publish forever
  while(ros::ok())
  {

      filter_target_state_msg.data=filtered_target_saved;
      filter_target_state_pub.publish(filter_target_state_msg);

      pub_input.publish(target_input);
      pub_filtered.publish(target_filtered);
      pub_filtered_target.publish(target_filtered_target);
      if(use_clustering){
        pub_cluster0.publish(target_cluster0);
        pub_cluster1.publish(target_cluster1);
        //pub_cluster2.publish(target_cluster2);
        //pub_cluster3.publish(target_cluster3);
        //pub_cluster4.publish(target_cluster4);
        pub_cluster_min.publish(target_cluster_min);
        target_marker_pub.publish(target_marker); // a marker around the target 
        cluster_markers_pub.publish(cluster_markers);
      }
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
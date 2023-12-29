/*
// This program applies a series of filters and processes to the input pointcloud
// to prepare for registration 
// A .pcd file is read as input and a new .pcd file is written as output
// A target .pcd file can also be input for cluster selection
// Tristan Hill - 07/14/2023
// updated - 12/18/2023 for PCDs from ds435i depth camera

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
typedef std::vector < PointCloudPtr, Eigen::aligned_allocator < PointCloudPtr > > PointCloudVec;
// aligned_allocator - STL compatible allocator to use with types requiring a non-standard alignment 

bool get_cloud_complete=0;
bool filtered_cloud_saved=0;

void get_cloud_stateCallback(const std_msgs::Bool::ConstPtr& msg){
  if (!msg->data){
    ROS_INFO("get_cloud in progress, waiting to begin filtering ...");
  }
  else if (msg->data&&!get_cloud_complete){
    ROS_INFO("get_cloud complete, beginning filtering");
    get_cloud_complete=1;
  }
}

// this function applies a bounding box and a voxel filter to the input cloud
void filter_cloud(PointCloud &input, PointCloud &output, double box[], double leaf_size, bool translate_output, bool auto_bounds){

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(input,*cloud);

  std::cout<<"Beginning filter_cloud() function" << std::endl;
  std::cout<<"Before filtering there are "<<cloud->width * cloud->height << " data points in the point cloud. "<< std::endl;
  
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
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
 
  // Define a translation the x,y,z directions
  transform_2.translation() << -box[0], -box[2], -box[4];

  // Print the transformation
  std::cout << "filter_cloud translate output transformation" << std::endl;
  std::cout << transform_2.matrix() << std::endl;

  // Execute the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);

  // save translated output or original output depending on config file
  if (translate_output){
    pcl::copyPointCloud(*transformed_cloud, output);
  }else{
    pcl::copyPointCloud(*cloud,output);
  }

}

// this function performs Euclidean cluster extraction to separate parts of the pointcloud 
PointCloudVec cluster_cloud(PointCloud &input,  int min_size, int max_size, double tolerance){

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(input,*cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance); // cluster parameters set in config file
  ec.setMinClusterSize (min_size);
  ec.setMaxClusterSize (max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
 
  // instatiate a std vector of pcl pointclouds with pcl PointXYZ points (see typedef above)
  PointCloudVec clusters;

  int j = 0;
  for (const auto& cluster : cluster_indices) 
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) { // add points to cluster cloud
      cloud_cluster->push_back((*cloud)[idx]);
    } 
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster); // add clusters to vector of clusters

    //std::cout << "PointCloud representing cluster"<<j<<" has "<< cloud_cluster->size() << " data points " << std::endl;
    j++; // increment the cluster counter
  }

  for (int i = 0; i < clusters.size(); i++){
      std::cout << "Point Cloud " << i << " has " << clusters[i]->size() << " Points " << std::endl;
  }

  std::cout<< "clusters size: "<< clusters.size() <<std::endl;
 
  return clusters;
   
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

  bbox_quaternion=bboxQuaternion; // copy to the output variables, these lines crash now that I am passing in a vector
  bbox_transform=bboxTransform;

  //return;  // this empty return is not needed ?

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


void transform_cloud(PointCloud &input, PointCloud &output, Eigen::Vector3f rotation, Eigen::Vector3f translation)
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

  // Execute the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::transformPointCloud (*cloud, *transformed_cloud, transform); 
   
  pcl::copyPointCloud(*transformed_cloud, output);
 
}


int main(int argc, char** argv)
{

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);
  
  // setup subcribers for get_cloud_state
  ros::Subscriber get_cloud_state_sub = node.subscribe("/get_cloud/get_cloud_state", 1000, get_cloud_stateCallback);

  // publisher for filter_cloud_state
  ros::Publisher filter_cloud_state_pub = node.advertise<std_msgs::Bool> ("/filter_cloud/filter_cloud_state", 1);
  
  std_msgs::Bool filter_cloud_state_msg;
  filter_cloud_state_msg.data=filtered_cloud_saved;


  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_cloud v1.6                              "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_cloud: loading configuration file       "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // there is only one cmd line arg and it is the name of the config file
  // this is not true, the params are loaded in the launch file
  // read the config file(yaml) feild to pick the data files and set parameters

  // find the path to the this package (seam_detection)
  std::string packagepath = ros::package::getPath("seam_detection");

  // boolen parameters 
  bool save_output, translate_output, automatic_bounds, use_clustering, new_scan, transform_input;
  node.getParam("save_output", save_output);
  node.getParam("translate_output", translate_output);
  node.getParam("automatic_bounds", automatic_bounds);
  node.getParam("use_clustering", use_clustering);
  node.getParam("new_scan", new_scan);
  node.getParam("transform_input", transform_input);

  // parameters that contain strings  
  std::string input_path, output_path, target_path, input_file, output_file, target_file; 
  node.getParam("filter_cloud/input_file", input_file);
  input_path=packagepath+'/'+input_file;
  node.getParam("filter_cloud/output_file", output_file);
  output_path=packagepath+'/'+output_file;
  node.getParam("filter_cloud/target_file", target_file);
  target_path=packagepath+'/'+target_file;

  // parameters that contain doubles 
  double voxel_leaf_size, cluster_tolerance;
  node.getParam("filter_cloud/voxel_leaf_size", voxel_leaf_size);
  node.getParam("filter_cloud/cluster_tolerance", cluster_tolerance);

  // parameters that contain ints
  int cluster_min_size, cluster_max_size;
  node.getParam("filter_cloud/cluster_min_size", cluster_min_size);
  node.getParam("filter_cloud/cluster_max_size", cluster_max_size);

  // parameters that contain vectors of doubles
  std::vector<double> filter_box_vec;
  double filter_box[6];

  std::vector<double> pre_rotation_vec, pre_translation_vec;
  Eigen::Vector3f pre_rotation, pre_translation;

  node.getParam("filter_cloud/pre_rotation",  pre_rotation_vec);
  node.getParam("filter_cloud/pre_translation",  pre_translation_vec);
  
  for(unsigned i=0; i < pre_rotation_vec.size(); i++){
    pre_rotation[i]=pre_rotation_vec[i]; // copy from std vector to eigen vector3f 
    pre_translation[i]=pre_translation_vec[i]; 
  }

  node.getParam("filter_cloud/filter_box",  filter_box_vec);
  for(unsigned i=0; i < filter_box_vec.size(); i++)
    filter_box[i]=filter_box_vec[i]; // copy from vector to array 
  
  // get additional parameters set in launch file
  bool saving_source=0;
  bool saving_target=0;
  
  //get flags saving_target, saving_source as command line args (not from config file)
  if (argc>1){
    std::string arg1(argv[1]);    
    if (!arg1.compare("saving_target")){
      saving_target=1;
    }
    if (!arg1.compare("saving_source")){
      saving_source=1;
    }
    std::cout<<"saving_target set: "<<saving_target<<" through cmd line"<<std::endl;
    std::cout<<"saving_source set: "<<saving_source<<" through cmd line"<<std::endl;
  }

  //std::cout<<"Debug0"<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                     filter_cloud: preparing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl;
  
  // instantiate some cloud pointers
  PointCloud::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_filtered_target (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr cloud_cluster0 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster1 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster2 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster3 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster4 (new pcl::PointCloud<pcl::PointXYZ>);  

  // wait for pointcloud from get_cloud
  while(!get_cloud_complete&&new_scan){
    ros::spinOnce(); // update topics while waiting, only if getting a new scan
  }
  
  std::cout << "Loading cloud input file:" << input_path << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, *cloud_input) == -1)
  {
      std::cout<<"Couldn't read cloud input file:"<<input_path;
      return (-1);
  }
  std::cout << "Loaded cloud input file: "<< input_path <<std::endl<<
    cloud_input->width * cloud_input->height << " Data points from "<< input_path << std::endl;
  
  std::cout << "Loading cloud target file:" << input_path << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_path, *cloud_filtered_target) == -1)
  {
      std::cout<<"Couldn't read cloud target file:"<<target_path;
      return (-1);
  }
  std::cout << "Loaded cloud target file: "<< target_path <<std::endl<<
    cloud_filtered_target->width * cloud_filtered_target->height << " Data points from "<< target_path << std::endl;  

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                    filter_cloud: processing pointcloud data        "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  // pre-translate the cloud (this is a patch imo, but it is useful. Alternatively use rotate_cloud.cpp)
  if (transform_input){
    transform_cloud(*cloud_input, *cloud_transformed, pre_rotation, pre_translation);
    std::cout<< "pretransform complete" << std::endl;
  }else{
    pcl::copyPointCloud(*cloud_input, *cloud_transformed);
    std::cout<< "pretransform skipped" << std::endl;
   }

  // Filter the LiDAR cloud with a bounding box and a voxel (downsampling)
  filter_cloud(*cloud_transformed,*cloud_filtered, filter_box, voxel_leaf_size, translate_output, automatic_bounds); 
  
  //Eigen::Quaternionf target_quaternion, cluster0_quaternion, cluster1_quaternion, cluster2_quaternion, cluster3_quaternion, cluster4_quaternion, marker_quaternion;
  //Eigen::Vector3f target_translation, cluster0_translation, cluster1_translation, cluster2_translation, cluster3_translation, cluster4_translation, marker_translation;
  //Eigen::Vector3f target_dimension, cluster0_dimension, cluster1_dimension, cluster2_dimension, cluster3_dimension, cluster4_dimension, marker_dimension;
  
  Eigen::Quaternionf target_quaternion, marker_quaternion;
  Eigen::Vector3f target_translation, marker_translation;
  Eigen::Vector3f target_dimension, marker_dimension;
  
  std::vector < Eigen::Quaternionf > cluster_quaternions; // vector of quaternions, maybe not the best solution... send me a better one, 2D array containing quats? eh...
  std::vector < Eigen::Vector3f > cluster_translations;   // these could be in a 2D array, but then the syntax would not match
  std::vector < Eigen::Vector3f > cluster_dimensions;

  std::vector <float> cluster_scores;                     // vector of scores for each cluster

  //double target_volume;
  //double target_aspect_ratio;
 
  PointCloudVec cloud_clusters;  // vector of pointclouds to store the separate clusters (see typedef above)
  int m; // number of clusters found 
 
  if(use_clustering){
    
    cloud_clusters=cluster_cloud(*cloud_filtered, cluster_min_size, cluster_max_size, cluster_tolerance);
    // use the vector of clusters, and bounding box data to find best cluster (min score) 
    // show the number points in each cluster, just to check  
    // and find the minimum bounding box for all clusters in vector clusters
    std::cout<< "cloud_clusters size: "<< cloud_clusters.size() <<std::endl;
    m=cloud_clusters.size();

    double score, score_min;

    for (int i = 0; i < cloud_clusters.size(); i++){
      
      // get the pose and size of the minimum bounding box for each cluster
      Eigen::Quaternionf cluster_quaternion; // this is a temp variable to get the eigen::quaternion from the function which will be added to quaternions vector
      Eigen::Vector3f cluster_translation, cluster_dimension; // these are also temp vars for the same purpose, there is probably a better way to do this ... 
      pcabox_cloud(*cloud_clusters[i], cluster_quaternion, cluster_translation, cluster_dimension);  // does not work (compiles but throws runtime error), pick up here! 
      cluster_quaternions.push_back(cluster_quaternion);  // add the temp variable to the vectors
      cluster_translations.push_back(cluster_translation); 
      cluster_dimensions.push_back(cluster_dimension); 

      // then get the score for each cluster
      double score; // tmp var to get return from function
       //score=100;
      score=score_cluster(*cloud_clusters[i], *cloud_filtered_target);
      cluster_scores.push_back(score); // add it to the vector of scores

      // print cluster information to the terminal to test
      std::cout << "cluster "<< i <<" number of points: " << cloud_clusters[i]->size() << std::endl;
      std::cout << "cluster "<< i <<" PCA box quaternion: ["<< cluster_quaternions[i].x()<<","
                                          << cluster_quaternions[i].y()<<","
                                          << cluster_quaternions[i].z()<<","
                                          << cluster_quaternions[i].w()<<"]"<<std::endl;                                    
      std::cout << "cluster "<< i <<" PCA box translation: ["<< cluster_translations[i][0] << "," 
                                          <<cluster_translations[i][1] << "," 
                                          <<cluster_translations[i][2] << "]" <<std::endl;
      std::cout << "cluster "<< i <<" PCA box dimension: ["<< cluster_dimensions[i][0] << "," 
                                          <<cluster_dimensions[i][1] << "," 
                                          <<cluster_dimensions[i][2] << "]" <<std::endl;
      std::cout << "cluster "<< i <<" Score: " <<cluster_scores[i]<< std::endl;

      if (i==0){ // always choose and save the first cluster
        score_min=cluster_scores[0];
        pcl::io::savePCDFileASCII (output_path, *cloud_clusters[0]);
        std::cout<<"cluster cloud written to:"<< output_path <<std::endl;
      }          // check all the rest for improved scores
      else if (cluster_scores[i]<score_min){
        score_min=cluster_scores[i];
        pcl::io::savePCDFileASCII (output_path, *cloud_clusters[i]);
        std::cout<<"cluster cloud written to:"<< output_path <<std::endl;
      }

    }

    std::cout<<"the lowest score is:"<<score_min<<std::endl;

  } 

  // save filtered cloud 
  if(save_output&&!use_clustering){
    pcl::io::savePCDFileASCII (output_path, *cloud_filtered); // this might be the wrong file saved
    std::cout<<"Filtered cloud written to:"<< output_path <<std::endl;
  }
  filtered_cloud_saved=1; // this should be moved?


  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                   filter_cloud: pointcloud processing complete     "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                   filter_cloud: preparing visualization            "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  ros::Publisher pub_input = node.advertise<PointCloud> ("/cloud_input", 1) ;
  ros::Publisher pub_transformed = node.advertise<PointCloud> ("/cloud_transformed", 1) ;
  ros::Publisher pub_filtered = node.advertise<PointCloud> ("/cloud_filtered", 1) ;
  ros::Publisher pub_filtered_target = node.advertise<PointCloud> ("/cloud_filtered_target", 1) ;
  
  cloud_input->header.frame_id = "base_link";
  cloud_transformed->header.frame_id = "base_link";
  cloud_filtered->header.frame_id = "base_link";
  cloud_filtered_target->header.frame_id = "base_link";

  std::vector<ros::Publisher> pub_clusters;
  
  std::cout<<"publisher instantiate complete"<<std::endl;
 

  for (int i=0; i<m; i++){

    std::stringstream cluster_name;
    cluster_name << "cloud_cluster" << i;
    pub_clusters.push_back(node.advertise<PointCloud>(cluster_name.str(), 0));
    cloud_clusters[i]->header.frame_id = "base_link";
  
  }

  std::cout<<"publisher advertise complete"<<std::endl;


  ros::Publisher target_marker_pub = node.advertise<visualization_msgs::Marker>( "target_marker", 0 );
  ros::Publisher cluster_markers_pub = node.advertise<visualization_msgs::MarkerArray>( "cluster_markers", 0 );
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

    cluster_marker.id = 999;

    target_marker.pose.orientation.x = target_quaternion.vec()[0];
    target_marker.pose.orientation.y = target_quaternion.vec()[1];
    target_marker.pose.orientation.z = target_quaternion.vec()[2];
    target_marker.pose.orientation.w = target_quaternion.w();

    target_marker.scale.x = target_dimension[0];
    target_marker.scale.y = target_dimension[1];
    target_marker.scale.z = target_dimension[2];

    target_marker.pose.position.x = target_translation[0];
    target_marker.pose.position.y = target_translation[1];
    target_marker.pose.position.z = target_translation[2];

    // array of markers for pointcloud clusters 
    cluster_marker.header.frame_id = "base_link";
    cluster_marker.header.stamp = ros::Time();
    cluster_marker.type = visualization_msgs::Marker::CUBE;
    cluster_marker.action = visualization_msgs::Marker::ADD;

    cluster_marker.color.a = 0.5; // Don't forget to set the alpha!
    cluster_marker.color.r = 255.0/255.0; // white ish PCA boxes look decent
    cluster_marker.color.g = 255.0/255.0;
    cluster_marker.color.b = 255.0/255.0;
    
    for(size_t i = 0; i < m; i++){   // this part should be easy to update for the vector of clusters

      marker_quaternion=cluster_quaternions[i];
      marker_translation=cluster_translations[i];
      marker_dimension=cluster_dimensions[i];

      cluster_marker.id = i;

      cluster_marker.pose.orientation.x = marker_quaternion.vec()[0];
      cluster_marker.pose.orientation.y = marker_quaternion.vec()[1];
      cluster_marker.pose.orientation.z = marker_quaternion.vec()[2];
      cluster_marker.pose.orientation.w = marker_quaternion.w();

      cluster_marker.scale.x = marker_dimension[0];
      cluster_marker.scale.y = marker_dimension[1];
      cluster_marker.scale.z = marker_dimension[2];

      cluster_marker.pose.position.x = marker_translation[0];
      cluster_marker.pose.position.y = marker_translation[1];
      cluster_marker.pose.position.z = marker_translation[2];

      cluster_markers.markers.push_back(cluster_marker); // add the marker to the marker array   
    }

  }
    
  std::cout<<"===================================================================="<<std::endl;
  std::cout<<"                        filter_cloud: complete                     "<<std::endl;
  std::cout<<"===================================================================="<<std::endl<<std::endl;

  //publish forever
  while(ros::ok())
  {
      filter_cloud_state_msg.data=filtered_cloud_saved;
      filter_cloud_state_pub.publish(filter_cloud_state_msg);

      pub_input.publish(cloud_input);
      pub_transformed.publish(cloud_transformed);
      pub_filtered.publish(cloud_filtered);
      pub_filtered_target.publish(cloud_filtered_target);

      if(use_clustering){
        target_marker_pub.publish(target_marker);
        cluster_markers_pub.publish(cluster_markers);
        for (int i=0; i<m; i++){
          pub_clusters[i].publish(cloud_clusters[i]);
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
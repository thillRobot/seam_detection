/*
// This program applies a bounding box and voxel downsampling filter to a pointcloud
// A .pcd file is read as input and a new .pcd file is written as output
// Tristan Hill - 06/21/2023

// see README.md or https://github.com/thillRobot/seam_detection for documentation

// this code is based on PCL example code: 
// https://pcl.readthedocs.io/en/latest/passthrough.html
// https://pcl.readthedocs.io/en/latest/cluster_extraction.html

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
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

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

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> 

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
 
#include "ros/package.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
//#include <teaser/point_cloud.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenCor;

// this function applies a bounding box and a voxel filter to the input cloud
void filter_cloud(PointCloud &input, PointCloud &output, double box[], double leaf_size, bool translate_output, bool auto_bounds)
{

  //double xmin, xmax, ymin, ymax, zmin, zmax;; // this could be done without these copies
  //xmin=box[0];xmax=box[1];                        // for now they are being used 
  //ymin=box[2];ymax=box[3];
  //zmin=box[4];zmax=box[5];
  //leaf_size=params[6];

  PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy of the target cloud
  pcl::copyPointCloud(input,*cloud);

  std::cout<<"BEGINNING CLOUD FILTERING" << std::endl;
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
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if (j==0){ // save the first five clusters for now, this could be improved
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


    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << j;
    //writer.write<pcl::PointXYZ> ("cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
    j++;
  }

}

// this function finds the minimum bounding box of cloud using PCA
// algorithm and code adapted from http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
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

  bbox_quaternion=bboxQuaternion;
  bbox_transform=bboxTransform;

  return;

}


int main(int argc, char** argv)
{

  ros::init(argc,argv,"seam_detection");
  ros::NodeHandle node;
  ros::Rate loop_rate(2);

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     Filter Cloud v1.6                              "<<endl;
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
  bool save_output, translate_output, automatic_bounds;
  node.getParam("save_output", save_output);
  node.getParam("translate_output", translate_output);
  node.getParam("automatic_bounds", automatic_bounds);

  // parameters that contain strings  
  std::string input_path, output_path, input_file, output_file; 
  node.getParam("input_file", input_file);
  input_path=packagepath+'/'+input_file;

  node.getParam("output_file", output_file);
  output_path=packagepath+'/'+output_file;


  // parameters that contain doubles
  double voxel_leaf_size;

  // parameters that contain vectors of doubles
  std::vector<double> filter_box_vec;
  double filter_box[6];
  
  node.getParam("filter_box",  filter_box_vec);
  for(unsigned i=0; i < filter_box_vec.size(); i++)
    filter_box[i]=filter_box_vec[i]; // copy from vector to array 
  node.getParam("voxel_leaf_size", voxel_leaf_size);


  // setup a tf for a 'searchbox' marker so we we can see it in RVIZ - maybe someday...
  // static tf::TransformBroadcaster br_searchbox;
  // tf::Transform tf_searchbox;

  std::cout<<"Debug0"<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                     Preparing Pointcloud Data                      "<<endl;
  std::cout<<"===================================================================="<<endl;

  
  // instantiate some clouds
  PointCloud::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZ>); 
  PointCloud::Ptr cloud_cluster0 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster1 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster2 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster3 (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_cluster4 (new pcl::PointCloud<pcl::PointXYZ>);  

  //std::cout<<"Debug1"<<endl;

  std::cout << "Loading image file: "<< input_path <<std::endl;


  // load the cloud from input file (.pcd)
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_path, *cloud_input) == -1)
  {
      std::cout<<"Couldn't read image file:"<<input_path;
      return (-1);
  }
  std::cout << "Loaded image file: "<< input_path <<std::endl<<
      cloud_input->width * cloud_input->height << " Data points from "<< input_path << std::endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                    Processing Pointcloud Data                      "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  // Filter the LiDAR cloud with a bounding box and a voxel (downsampling)
  filter_cloud(*cloud_input,*cloud_output, filter_box, voxel_leaf_size, translate_output, automatic_bounds); 

  // find clusters in filtered cloud
  cluster_cloud(*cloud_output, *cloud_cluster0, *cloud_cluster1, *cloud_cluster2, *cloud_cluster3, *cloud_cluster4);

  // find the minimum bounding box for a single cluster
  Eigen::Quaternionf pcabox_quaternion0, pcabox_quaternion1, pcabox_quaternion2, pcabox_quaternion3, marker_quaternion;
  Eigen::Vector3f pcabox_translation0, pcabox_translation1, pcabox_translation2, pcabox_translation3, marker_translation;
  Eigen::Vector3f pcabox_dimensions0, pcabox_dimensions1, pcabox_dimensions2, pcabox_dimensions3, marker_dimensions;

  pcabox_cloud(*cloud_cluster0, pcabox_quaternion0, pcabox_translation0, pcabox_dimensions0);
  pcabox_cloud(*cloud_cluster1, pcabox_quaternion1, pcabox_translation1, pcabox_dimensions1);
  pcabox_cloud(*cloud_cluster2, pcabox_quaternion2, pcabox_translation2, pcabox_dimensions2);
  pcabox_cloud(*cloud_cluster3, pcabox_quaternion3, pcabox_translation3, pcabox_dimensions3); 

  std::cout<<"pcabox quaternion 0: "<<pcabox_quaternion0.w()<<std::endl;
  std::cout<<"pcabox rotation quaternion 0: "<<pcabox_quaternion0.vec()[0]<<","<<pcabox_quaternion0.vec()[1]<<","<<pcabox_quaternion0.vec()[2]<<","<<std::endl;
  std::cout<<"pcabox translation 0: "<<pcabox_translation0<<std::endl;

  // save filtered cloud 
  if(save_output){
    pcl::io::savePCDFileASCII (output_path, *cloud_output);
    std::cout<<"Filtered cloud written to:"<< output_path <<std::endl;
  }

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   Pointcloud Processing Complete                   "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                   Preparing Visualization                          "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  ros::Publisher pub_input = node.advertise<PointCloud> ("/cloud_input", 1) ;
  ros::Publisher pub_output = node.advertise<PointCloud> ("/cloud_output", 1) ;
  ros::Publisher pub_cluster0 = node.advertise<PointCloud> ("/cloud_cluster0", 1) ;
  ros::Publisher pub_cluster1 = node.advertise<PointCloud> ("/cloud_cluster1", 1) ;
  ros::Publisher pub_cluster2 = node.advertise<PointCloud> ("/cloud_cluster2", 1) ;
  ros::Publisher pub_cluster3 = node.advertise<PointCloud> ("/cloud_cluster3", 1) ;
  ros::Publisher pub_cluster4 = node.advertise<PointCloud> ("/cloud_cluster4", 1) ;

  cloud_input->header.frame_id = "base_link";
  cloud_output->header.frame_id = "base_link";
  cloud_cluster0->header.frame_id = "base_link";
  cloud_cluster1->header.frame_id = "base_link";
  cloud_cluster2->header.frame_id = "base_link";
  cloud_cluster3->header.frame_id = "base_link";
  cloud_cluster4->header.frame_id = "base_link";

  ros::Publisher pcabox_markers_pub = node.advertise<visualization_msgs::MarkerArray>( "pcabox_markers", 0 );
  visualization_msgs::MarkerArray pcabox_markers;
  visualization_msgs::Marker pcabox_marker;

  pcabox_marker.header.frame_id = "base_link";
  pcabox_marker.header.stamp = ros::Time();
  pcabox_marker.type = visualization_msgs::Marker::CUBE;
  pcabox_marker.action = visualization_msgs::Marker::ADD;

  pcabox_marker.color.a = 0.15; // Don't forget to set the alpha!
  pcabox_marker.color.r = 255.0/255.0;
  pcabox_marker.color.g = 0.0/255.0;
  pcabox_marker.color.b = 255.0/255.0;
  
  for(size_t i = 0; i < 4; i++){  

    if (i==0){
      marker_quaternion=pcabox_quaternion0;
      marker_translation=pcabox_translation0;
      marker_dimensions=pcabox_dimensions0;
    }else if(i==1){
      marker_quaternion=pcabox_quaternion1;
      marker_translation=pcabox_translation1;
      marker_dimensions=pcabox_dimensions1;
    }else if(i==2){
      marker_quaternion=pcabox_quaternion2;
      marker_translation=pcabox_translation2;
      marker_dimensions=pcabox_dimensions2;
    }else if(i==3){
      marker_quaternion=pcabox_quaternion3;
      marker_translation=pcabox_translation3;
      marker_dimensions=pcabox_dimensions3;
    }

    pcabox_marker.id = i;

    pcabox_marker.pose.orientation.x = marker_quaternion.vec()[0];
    pcabox_marker.pose.orientation.y = marker_quaternion.vec()[1];
    pcabox_marker.pose.orientation.z = marker_quaternion.vec()[2];
    pcabox_marker.pose.orientation.w = marker_quaternion.w();

    pcabox_marker.scale.x = marker_dimensions[0];
    pcabox_marker.scale.y = marker_dimensions[1];
    pcabox_marker.scale.z = marker_dimensions[2];

    pcabox_marker.pose.position.x = marker_translation[0];
    pcabox_marker.pose.position.y = marker_translation[1];
    pcabox_marker.pose.position.z = marker_translation[2];

    pcabox_markers.markers.push_back(pcabox_marker); // add the marker to the marker array   
  }
  

  std::cout<<"===================================================================="<<endl;
  std::cout<<"                        seam_detection Complete                     "<<endl;
  std::cout<<"===================================================================="<<endl<<endl;

  //publish forever
  while(ros::ok())
  {

      pub_input.publish(cloud_input);
      pub_output.publish(cloud_output);
      pub_cluster0.publish(cloud_cluster0);
      pub_cluster1.publish(cloud_cluster1);
      pub_cluster2.publish(cloud_cluster2);
      pub_cluster3.publish(cloud_cluster3);
      pub_cluster4.publish(cloud_cluster4);
      pcabox_markers_pub.publish(pcabox_markers);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}



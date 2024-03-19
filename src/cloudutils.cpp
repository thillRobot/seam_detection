/*

 cloudutils - utility functions for working with pointsclouds, PCL, and ROS
 Tristan Hill - 03/03/2024
 see README.md or https://github.com/thillRobot/seam_detection for documentation

*/

#include "cloudutils.h"
#include <string> 
#include <boost/thread/thread.hpp>
#include <thread>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry> 

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
// PCL PointCloud with XYZ RGB Normal Points
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;

// DEFINITIONS

// default constructor

CloudUtils::CloudUtils() 
  : config("cloudutils"), pub_idx(0){
  
  // find the path to the this package (seam_detection)
  package_path = ros::package::getPath("seam_detection");

}


CloudUtils::CloudUtils(std::string cfg="cloudutils", int idx=0) 
  : config(cfg), pub_idx(idx){
}

CloudUtils::~CloudUtils(){
}


void CloudUtils::loadConfig(std::string cfg){

  config=cfg;
  
  node.getParam("input_file", input_file);    
  node.getParam("output_file", output_file);
 
} 

std::string CloudUtils::getConfig(void){

  return config;

}


// templated function to load pcl::PointCloud<point_t> from PCD file as defined in config
template <typename point_t>
int CloudUtils::loadCloud(pcl::PointCloud<point_t> &input, std::string file) {

  std::cout<<"|---------- CloudUtils::LoadCloud - loading PCD file ----------|"<<std::endl;

  std::string path;
  path=package_path+"/"+file;
  //path=file;

  std::cout << "Loading input pointcloud file: " << path << std::endl;
  if (pcl::io::loadPCDFile<point_t> (path, input) == -1)
  {
    std::cout<<"Failed to load input pointcloud file: "<< path <<std::endl;
    return (-1);
  }
  std::cout << "Loaded "<<input.width * input.height << " data points from input pointcloud file: "<< path <<std::endl;
  return 0;

}

template int CloudUtils::loadCloud<pcl::PointXYZRGB>
              (pcl::PointCloud<pcl::PointXYZRGB> &input, std::string file);


// templated function to save pcl::PointCloud<point_t> to PCD file as defined in config
template <typename point_t>
int CloudUtils::saveCloud(pcl::PointCloud<point_t> &output, std::string file){
  // the name output here is very confusing, consider changing this
  std::cout<<"|---------- FilterDataset::SaveCloud - saving PCD file ----------|"<<std::endl;

  std::string path;
  path=package_path+"/"+file;
  //path=file;
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
 
template int CloudUtils::saveCloud<pcl::PointXYZRGB>
            ( pcl::PointCloud<pcl::PointXYZRGB> &output, std::string file );   


// templated function to publish a single pcl::PointCloud<point_t> as a ROS topic 
template <typename point_t>
void CloudUtils::publishCloud(pcl::PointCloud<point_t> &cloud, std::string topic, std::string frame){
  std::cout<<"|---------- SeamDetection::publishCloud - publishing single cloud ----------|"<<std::endl;

  // advertise a new topic and publish a msg each time this function is called
  pub_clouds.push_back(node.advertise<pcl::PointCloud<point_t>>(topic, 0, true));

  cloud.header.frame_id = frame; // reference to show cloud in rviz

  pub_clouds[pub_clouds.size()-1].publish(cloud);

  ros::spinOnce();

}

// declare all possible types used with the template here 
template void CloudUtils::publishCloud< pcl::PointXYZRGB >
              (pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::string topic, std::string frame);

template void CloudUtils::publishCloud< pcl::PointXYZRGBNormal >
              (pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, std::string topic, std::string frame);


// function to publish a vector of PointClouds representing clusters as a ROS topic
void CloudUtils::publishClusters(PointCloudVec &clusters, std::string prefix){
  std::cout<<"|---------- CloudUtils::publishClusters - publishing clusters ----------|"<<std::endl;
  std::cout<<"|---------- overloaded for `PointCloudVec` ----------|"<<std::endl;

  std::cout<<"clusters size: "<<clusters.size()<<std::endl;

  for (int i=0; i<clusters.size(); i++){
    // advertise a topic and publish a msg for each cluster in clusters
    std::stringstream name;
    name << prefix << i;
    pub_clusters.push_back(node.advertise<PointCloud>(name.str(), 0, true));
    clusters[i]->header.frame_id = "base_link";
    pub_clusters[pub_idx].publish(clusters[i]);
    pub_idx++;
  }

  ros::spinOnce();
}


// overloaded function to publish a vector of PointClouds with normals representing clusters as a ROS topic
void CloudUtils::publishClusters(PointCloudNormalVec &clusters, std::string prefix){
  std::cout<<"|---------- CloudUtils::publishClusters - publishing clusters ----------|"<<std::endl;
  std::cout<<"|---------- overloaded for `PointCloudNormalVec` ----------|"<<std::endl;

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
void CloudUtils::publishClustersT(std::vector< typename pcl::PointCloud<point_t>::Ptr,
        Eigen::aligned_allocator< typename pcl::PointCloud<point_t>::Ptr> > &clusters, std::string prefix){
  std::cout<<"|---------- CloudUtils::publishClustersT - publishing clusters ----------|"<<std::endl;
  std::cout<<"|---------- templated for `PointCloudVec<pcl::PointCloud<point_t>>` ----------|"<<std::endl;

  for (int i=0; i<clusters.size(); i++){
    // advertise a topic and publish a msg for each cluster in clusters
    std::stringstream name;
    name << prefix << i;
    pub_clusters.push_back(node.advertise<pcl::PointCloud<point_t>>(name.str(), 0, true)); // this type needs handling too
    clusters[i]->header.frame_id = "base_link";
    pub_clusters[pub_idx].publish(clusters[i]);
    pub_idx++;
  }

  ros::spinOnce();
}

template void CloudUtils::publishClustersT<pcl::PointXYZRGB>
              (std::vector< typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               Eigen::aligned_allocator<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > &clusters, std::string prefix);

// function to copy PointCloud with XYZRGB points - not needed, use pcl::copyPointCloud()
void CloudUtils::copyCloud(PointCloud &input, PointCloud &output){

  std::cout<<"the point cloud input has "<< input.size()<< " points"<<std::endl;
  for (int i=0; i<input.size(); i++) { // add points to cluster cloud
    output.push_back(input[i]);
  }
  std::cout<<"the point cloud output has "<< output.size()<< " points"<<std::endl;

}


// function to return the median value of a std::vector
// it seems like there would be a std method for this
double CloudUtils::getMedian(std::vector<double> vals){

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


// overloaded function to return the median value of a Eigen::VectorXd (dynxamic sized vector of doubles)
double CloudUtils::getMedian(Eigen::VectorXd vals){

  int size=vals.size();

  if (size==0){
    return 0; // size 0 vector has no median
  }else{
    std::sort(vals.data(), vals.data()+vals.size());
    if(size%2==0){
      return (vals[size/2-1]+vals[size/2])/2;
    }else{
      return vals[size/2];
    }
  }
}

/*
// function to return the median value of a std::vector
// it seems like there would be a std method for this
std::vector<double> CloudUtils::getMedianColor(pcl::PointCloud<pcl::PointXYZRGB> &input ){

  //size_t size=vals.size();

  // term 4 - color metric
  //std::uint32_t red, green, blue;
  //int i = 1;
  double red_med, green_med, blue_med;
  //red=input.points[10].r;
  //blue=input.points[10].g;
  //green=input.points[10].b;

  int size=input.size();
  
  std::vector<double> reds, blues, greens;

  
  for (int i; i<=size; i++){
    reds.push_back(input.points[10].r);
    greens.push_back(input.points[10].g);
    blues.push_back(input.points[10].b);
    
  }
  int k=11;
  //std::cout<<"cloud [red, green ,blue] values : ["
  //         <<reds[k]<<", "<<greens[k]<<", "<<blues[k]<<" ]"<<std::endl; 
  
  red_med=getMedian(reds);
  green_med=getMedian(greens);
  blue_med=getMedian(blues);

  std::vector<double> rgb_meds;
  rgb_meds.push_back(red_med);
  rgb_meds.push_back(green_med);
  rgb_meds.push_back(blue_med);

  //std::cout<<"cloud [red, green ,blue] median values : ["
  //         <<red_med<<", "<<green_med<<", "<<blue_med<<" ]"<<std::endl;
  return rgb_meds;
}
*/


// overloaded function to return the median rgb values of a pcl::PointCloud<pcl::PointXYZRGB>
// this returns an Eigen::VectorXd containing the three median color vals
Eigen::VectorXd CloudUtils::getMedianColor(pcl::PointCloud<pcl::PointXYZRGB> &input ){

  //size_t size=vals.size();

  double red_med, green_med, blue_med;

  int size=input.size();
  
  std::vector<double> reds, blues, greens;

  for (int i=0; i<=size; i++){
    reds.push_back(input.points[i].r);
    greens.push_back(input.points[i].g);
    blues.push_back(input.points[i].b);   
  }
  
  red_med=getMedian(reds);
  green_med=getMedian(greens);
  blue_med=getMedian(blues);
  
  Eigen::VectorXd rgb_meds(3);
  rgb_meds(0)=red_med;
  rgb_meds(1)=green_med;
  rgb_meds(2)=blue_med;
  
  return rgb_meds;
}


// function to merge two pointclouds into a single pointcloud, same function as getCloudUnion()
void CloudUtils::mergeClouds(PointCloud &cloud1, PointCloud &cloud2, PointCloud &output){

  pcl::copyPointCloud(cloud1, output); // start with a copy of cloud1
  for (int i=0; i<cloud2.size(); i++){
      // add all the points from cloud2
      output.push_back(cloud2.points[i]);
  }

  std::cout<< "the merged cloud has "<< output.size() << " points" <<std::endl;
}


// function to merge a vector of pointclouds into a single pointcloud
void CloudUtils::mergeClusters(PointCloudVec &clusters, PointCloud &output){

  for (int i=0; i<clusters.size(); i++){
  
    for (int j=0; j<clusters[i]->size(); j++){
      output.push_back(clusters[i]->points[j]);
    }
  
  }

  std::cout<< "the merged cloud has "<< output.size() << " points" <<std::endl;
}



// overloaded function to merge a vector of pointclouds and return pointer to single pointcloud 
PointCloud::Ptr CloudUtils::mergeClusters(PointCloudVec &clusters){

  PointCloud::Ptr output (new PointCloud);

  for (int i=0; i<clusters.size(); i++){
  
    for (int j=0; j<clusters[i]->size(); j++){
      output->push_back(clusters[i]->points[j]);
    }
  
  }

  std::cout<< "the merged cloud has "<< output->size() << " points" <<std::endl;
  return output;
}


// function to merge two pointclouds into a single pointcloud
void CloudUtils::getCloudUnion(PointCloud &cloud1, PointCloud &cloud2, PointCloud &output){

  pcl::copyPointCloud(cloud1, output); // start with a copy of cloud1
  for (int i=0; i<cloud2.size(); i++){
      // add all the points from cloud2
      output.push_back(cloud2.points[i]);
  }

  std::cout<< "the union cloud has "<< output.size() << " points" <<std::endl;
}

// function to find the intersection cloud3 of clouds1 and cloud2 defined by the points in cloud 1 AND cloud 2
// this is based on exact comparison and will not work for approximate cloud points 
void CloudUtils::getCloudIntersection(PointCloud &cloud1, PointCloud &cloud2, PointCloud &cloud3){

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

// overloaded function to find and return intersection of clouds1 and cloud2 defined by the points in cloud 1 AND cloud 2
// this is based on exact comparison and will not work for approximate cloud points 
PointCloud::Ptr CloudUtils::getCloudIntersection(PointCloud &cloud1, PointCloud &cloud2){

  PointCloud::Ptr cloud3 (new PointCloud);

  for (int i=0; i<cloud1.size(); i++) { // add points to cluster cloud
    for (int j=0; j<cloud2.size(); j++){
      // check if all three coordinate values are the same
      if (cloud1.points[i].x==cloud2.points[j].x&&cloud1.points[i].y==cloud2.points[j].y&&cloud1.points[i].z==cloud2.points[j].z){
        cloud3->push_back(cloud1[i]); // add the shared point to the new cloud
      }
    }
  }
  std::cout<< "the intersection cloud has "<< cloud3->size() << " points" <<std::endl;
  return cloud3;
}


// function to find cluster of clouds as the intersection of two matching clusters, getCloudIntersection()   
PointCloudVec CloudUtils::getClusterIntersections(PointCloudVec &clusters1, PointCloudVec &clusters2, int thresh){
  
  PointCloud::Ptr cloud (new PointCloud); // tmp cloud
  PointCloudVec cloud_intersections;

  for(int i=0; i<clusters1.size(); i++){

    getCloudIntersection(*clusters1[i], *clusters2[i], *cloud); // find the points in clusters1[i] AND clusters2[j]

    if (cloud->size()>thresh){ // check if the intersection passes a threshold
      std::cout<<"test"<<i<<", cluster1["<<i<<"] intersected with cluster2["<<i<<"] has "<<cloud->size()
               <<" points and will be added to the intersection cluster"<<std::endl;
                                            
      PointCloud::Ptr cluster (new PointCloud); // allocate memory for the pointcloud to be stored and pointed to by the new PointCloudVec 
      pcl::copyPointCloud(*cloud, *cluster);  // make a copy to avoid the clear below

      cloud_intersections.push_back(cluster); // add the intersection to the cluster of intersections
      std::cout<<"the added cluster has "<<cluster->size()<<" points"<<std::endl;

    }else{
      std::cout<<"test"<<i<<", cluster1["<<i<<"] intersected with cluster2["<<i<<"] has "<<cloud->size()
               <<" points and will NOT be added to the intersection cluster"<<std::endl;
    }
    cloud->clear();

  }
  return cloud_intersections;
}


// function to find cluster of clouds as the intersection of two matching clusters, getCloudIntersection()   
PointCloudVec CloudUtils::getClusterUnions(PointCloudVec &clusters1, PointCloudVec &clusters2, int thresh){
  
  PointCloud::Ptr cloud (new PointCloud); // tmp cloud
  PointCloudVec cloud_unions;

  for(int i=0; i<clusters1.size(); i++){

    getCloudUnion(*clusters1[i], *clusters2[i], *cloud); // find the points in clusters1[i] AND clusters2[j]

    if (cloud->size()>thresh){ // check if the union passes a threshold
      std::cout<<"test"<<i<<", cluster1["<<i<<"] intersected with cluster2["<<i<<"] has "<<cloud->size()
               <<" points and will be added to the intersection cluster"<<std::endl;
      
      // allocate memory for the pointcloud to be stored and pointed to by the new PointCloudVec                                      
      PointCloud::Ptr cluster (new PointCloud); 
      pcl::copyPointCloud(*cloud, *cluster);  // make a copy to avoid the clear below

      cloud_unions.push_back(cluster); // add the intersection to the cluster of intersections
      std::cout<<"the added cluster has "<<cluster->size()<<" points"<<std::endl;

    }else{
      std::cout<<"test"<<i<<", cluster1["<<i<<"] intersected with cluster2["<<i<<"] has "<<cloud->size()
               <<" points and will NOT be added to the intersection cluster"<<std::endl;
    }
    cloud->clear();

  }
  return cloud_unions;
}


// function to find cluster of clouds as the  intersection of two clusters, calls SeamDetection::getClusterIntersection()   
// this is a bit confusing and an explanation would help
void CloudUtils::getClusterIntersectionAll(PointCloudVec &clusters1, PointCloudVec &clusters2, PointCloudVec &clusters3, int thresh){

  PointCloud::Ptr cloud (new PointCloud);
  //PointCloudVec clusters;

  int k=0; // comparison counter (counts each time)
  for(int i=0; i<clusters1.size(); i++){ // for each cluster in clusters1

    for (int j=0; j<clusters2.size(); j++){ // compare with each cluster in clusters2

      getCloudIntersection(*clusters1[i], *clusters2[j], *cloud); // find the points in clusters1[i] AND clusters2[j]

      if (cloud->size()>thresh){ // check if the intersection passes a threshold
        std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "
                 <<cloud->size()<<" points and will be added to the intersection cluster"<<std::endl;
        //clusters.push_back(cloud); // add the intersection to the cluster of intersections
        clusters3.push_back(cloud);
      }else{
        std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "
                 <<cloud->size()<<" points and will NOT be added to the intersection cluster"<<std::endl;
      }
      cloud->clear(); // empty the tmp cloud for the next intersection
      k++;
    }
  }

  std::cout<<"there are "<<clusters3.size()<<" clouds in the cluster intersection"<< std::endl;
  //return clusters;
}


// function to find cluster of clouds as the intersection of two clusters, calls SeamDetection::getClusterIntersection()   
PointCloudVec CloudUtils::getClusterIntersectionAll(PointCloudVec &clusters1, PointCloudVec &clusters2, int thresh){

  PointCloud::Ptr cloud (new PointCloud); // tmp memory for kth test intersection 
  PointCloudVec clusters;

  int k=0; // comparison counter (counts each time)
  for(int i=0; i<clusters1.size(); i++){ // for each cluster in clusters1

    for (int j=0; j<clusters2.size(); j++){ // compare with each cluster in clusters2

      getCloudIntersection(*clusters1[i], *clusters2[j], *cloud); // find the points in clusters1[i] AND clusters2[j]

      if (cloud->size()>thresh){ // check if the intersection passes a threshold
        std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "
                 <<cloud->size()<<" points and will be added to the intersection cluster"<<std::endl;

        // allocate memory for the pointcloud to be stored and pointed to by the new PointCloudVec  (vector of pointcloud pointers)
        PointCloud::Ptr cluster (new PointCloud);
        pcl::copyPointCloud(*cloud, *cluster); // make a copy to avoid the clear below

        //be careful to avoid adding the same cluster multiple time
        //intersection 'cluster' is unique, new clusters should not have repeat entries
        clusters.push_back(cluster); // add the intersection to the cluster of intersections
        //std::cout<<"the added cluster has "<<clusters[clusters.size()-1]->size()<<" points"<<std::endl; 
        std::cout<<"the added cluster has "<<cluster->size()<<" points"<<std::endl;

      }else{
        std::cout<<"test"<<k<<", cluster1["<<i<<"] intersected with cluster2["<<j<<"] has "
                 <<cloud->size()<<" points and will NOT be added to the intersection cluster"<<std::endl;
      }
      cloud->clear(); // empty the tmp cloud for the next intersection, is this clear wiping both??? YES INDEED ! BUG IS HERE!
      std::cout<<"the added cluster has "<<clusters[clusters.size()-1]->size()<<" points after the clear"<<std::endl;
      k++;
    }
  }
  
  std::cout<<"there are "<<clusters.size()<<" clouds in the cluster intersection"<< std::endl;
  return clusters;

}


// function to trim point cloud to a given size, primarily to keep teaser from crashing 
template <typename point_t>
void CloudUtils::trimCloud(pcl::PointCloud<point_t> &input, pcl::PointCloud<point_t> &output, int output_size){

  typename pcl::PointCloud<point_t>::Ptr cloud (new pcl::PointCloud<point_t>);
     
  int input_size=input.size();
  if (input_size>output_size){
  
    for (int i=0; i<input_size; i++){

      output.push_back(input.points[i]);  

    } 
  
  }

  std::cout<<"trimmed pointcloud has "<<output.size()<<" points"<<std::endl;
  
}

template void CloudUtils::trimCloud<pcl::PointXYZRGB>
              (pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output, int output_size);


template<typename point_t>
// function to find the minimum oriented bounding box of a cloud using principle component analysis
void CloudUtils::getPCABox(pcl::PointCloud<point_t> &input, 
                           Eigen::Quaternionf& rotation, 
                           Eigen::Vector3f& translation, 
                           Eigen::Vector3f& dimension){

  typename pcl::PointCloud<point_t>::Ptr cloud (new pcl::PointCloud<point_t>); //allocate memory 
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
  typename pcl::PointCloud<point_t>::Ptr projected_cloud (new pcl::PointCloud<point_t>);

  pcl::transformPointCloud(*cloud, *projected_cloud, projection_transform);
  // Get the minimum and maximum points of the transformed cloud.
  point_t min_point, max_point;
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
 
template void CloudUtils::getPCABox<pcl::PointXYZRGB>
                                   (pcl::PointCloud<pcl::PointXYZRGB> &input, 
                                    Eigen::Quaternionf& rotation, 
                                    Eigen::Vector3f& translation, 
                                    Eigen::Vector3f& dimension);

template void CloudUtils::getPCABox<pcl::PointXYZRGBNormal>
                                   (pcl::PointCloud<pcl::PointXYZRGBNormal> &input, 
                                    Eigen::Quaternionf& rotation, 
                                    Eigen::Vector3f& translation, 
                                    Eigen::Vector3f& dimension);

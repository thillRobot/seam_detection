#ifndef CLOUDUTILS_H
#define CLOUDUTILS_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>


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


class CloudUtils
{

  private:
    // PRIVATE functions
    // PRIVATE attributes    
    
    ros::NodeHandle node;
    
    std::string config, input_file, output_file;
    
    // generic publisher, can this be used for all of the clouds?
    std::vector<ros::Publisher> pub_clouds;
    std::vector<ros::Publisher> pub_clusters;
    int pub_idx;
  
  public:  
    // PUBLIC functions

    CloudUtils(); 
    CloudUtils(std::string, int);
    ~CloudUtils();  

    void loadConfig(std::string cfg);

    std::string getConfig(void);

    template <typename point_t>
    int loadCloud(pcl::PointCloud<point_t> &input, std::string file);
   
    template <typename point_t> 
    void publishCloud(pcl::PointCloud<point_t> &cloud, std::string topic, std::string frame);

    void publishClusters(PointCloudVec &clusters, std::string prefix);
    void publishClusters(PointCloudNormalVec &clusters, std::string prefix);
    template <typename point_t>  // templated publishClusters not currently working, can deduce template type ...
    void publishClustersT(const std::vector<typename pcl::PointCloud<point_t>::Ptr,
                    Eigen::aligned_allocator<typename pcl::PointCloud<point_t>::Ptr> > &clusters, std::string prefix);   
    
    void copyCloud(PointCloud &input, PointCloud &output);

    double getMedian(std::vector<double> vals);
    double getMedian(Eigen::VectorXd vals);
   
    void mergeClusters(PointCloudVec &clusters, PointCloud &output);
    PointCloud::Ptr mergeClusters(PointCloudVec &clusters);
    
    template <typename point_t>
    int saveCloud(pcl::PointCloud<point_t> &output, std::string file);
  
    // PUBLIC attributes

    std::string package_path;

};


#endif

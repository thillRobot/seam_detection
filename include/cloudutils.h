#ifndef CLOUDUTILS_H
#define CLOUDUTILS_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
// PCL PointClouds with XYZ RGB Normal Points
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;


class CloudUtils
{

  private:
    // PRIVATE functions
    // PRIVATE attributes    
    
    ros::NodeHandle node;
    
    std::string config, input_file, output_file;
    
    // generic publisher, can this be used for all of the clouds?
    std::vector<ros::Publisher> pub_clouds;

  public:  
    // PUBLIC functions

    CloudUtils(); 
    CloudUtils(std::string);
    ~CloudUtils();  

    void loadConfig(std::string cfg);

    std::string getConfig(void);

    template <typename point_t>
    int loadCloud(pcl::PointCloud<point_t> &input, std::string file);
   
    template <typename point_t> 
    void publishCloud(pcl::PointCloud<point_t> &cloud, std::string topic, std::string frame);
    
    // PUBLIC attributes

    std::string package_path;

};


#endif

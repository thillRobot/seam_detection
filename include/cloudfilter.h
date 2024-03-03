#ifndef CLOUDFILTER_H
#define CLOUDFILTER_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

// PCL PointClouds with XYZ RGB Points
//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
// PCL PointClouds with XYZ RGB Normal Points
//typedef pcl::PointXYZRGBNormal PointNT;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;


class CloudFilter
{

  private:
    // PRIVATE functions
    // PRIVATE attributes    
    
    ros::NodeHandle node;
    
    bool auto_bounds;
    std::string config, input_file, output_file;
    std::vector<double> bounding_box;  

  public:  
    // PUBLIC functions

    CloudFilter(); 
    CloudFilter(std::string);
    ~CloudFilter();  

    void loadConfig(std::string cfg);

    std::string getConfig(void);

    std::vector<double> getBoundingBox(void);

    void boundCloud(PointCloud &input, PointCloud &output, std::vector<double> box);

    void transformCloud(PointCloud &input, PointCloud &output, std::vector<double> rotation, std::vector<double> translation);
    void transformCloud(PointCloud &input, PointCloud &output, Eigen::Vector3d rotation, Eigen::Vector3d translation);    
    void transformCloud(PointCloud &input, PointCloud &output, Eigen::Quaterniond rotation, Eigen::Vector3d translation);
    
    //void smoothCloud(PointCloud &input, PointCloudNormal &output);

    template <typename point_t, typename point_normal_t> 
    void smoothCloud(pcl::PointCloud<point_t> &input, pcl::PointCloud<point_normal_t> &output);
   
    template <typename point_t>
    void downsampleCloud(pcl::PointCloud<point_t> &input, pcl::PointCloud<point_t> &output, double leaf_size);

    void extractPolygonalPrism(PointCloud &input);
    // PUBLIC attributes

};


#endif

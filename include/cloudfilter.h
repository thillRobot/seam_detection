#ifndef CLOUDFILTER_H
#define CLOUDFILTER_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

// PCL PointClouds with XYZ RGB Points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;


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

    void boundCloud(PointCloud &input, PointCloud &output);


    // PUBLIC attributes
    
};


#endif

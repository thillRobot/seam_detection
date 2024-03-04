#ifndef CLOUDREGISTRATION_H
#define CLOUDREGISTRATION_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

 #include <tf/transform_broadcaster.h>
 #include <tf/transform_listener.h>

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


class CloudRegistration
{

  private:
    // PRIVATE functions
    // PRIVATE attributes    
    
    ros::NodeHandle node;
    
    std::string config, input_file, output_file;
    std::string package_path;    
  
  public:  
    // PUBLIC functions

    CloudRegistration(); 
    CloudRegistration(std::string, int);
    ~CloudRegistration();  

    void loadConfig(std::string cfg);

    std::string getConfig(void);

    //PUBLIC attributes
    double register_cloud_icp(PointCloud &source, PointCloud &target, tf::StampedTransform &T_AB, tf::StampedTransform &T_BA,
                           geometry_msgs::TransformStamped &msg_AB, geometry_msgs::TransformStamped &msg_BA,
                           double max_corr_dist, double max_iter, double trns_epsl, double ecld_fitn_epsl,
                           double ran_rej_thrsh, double e_results[],double c_offset[]);

};


#endif

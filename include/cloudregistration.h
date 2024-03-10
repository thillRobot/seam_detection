#ifndef CLOUDREGISTRATION_H
#define CLOUDREGISTRATION_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
 
#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>

// PCL PointClouds with XYZ RGB Points
//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
// PCL PointClouds with XYZ RGB Normal Points
//typedef pcl::PointXYZRGBNormal PointNT;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;
//typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PointCloudNormalPtr;
// Vector of PointClouds
// aligned_allocator - STL compatible allocator to use with types requiring a non-standard alignment 
//typedef std::vector < PointCloud::Ptr, Eigen::aligned_allocator < PointCloud::Ptr > > PointCloudVec;
//typedef std::vector < PointCloudNormal::Ptr, Eigen::aligned_allocator < PointCloudNormal::Ptr > > PointCloudNormalVec;


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

    template <typename point_t>
    double registerCloudICP(pcl::PointCloud<point_t> &source, 
                            pcl::PointCloud<point_t> &target, 
                            tf::StampedTransform &T_AB, 
                            tf::StampedTransform &T_BA,
                            geometry_msgs::TransformStamped &msg_AB, 
                            geometry_msgs::TransformStamped &msg_BA);
    
    template <typename point_t>    
    void registerCloudTeaser(pcl::PointCloud<point_t> &source, 
                             pcl::PointCloud<point_t> &target,
                             tf::StampedTransform &T_AB, 
                             tf::StampedTransform &T_BA,
                             geometry_msgs::TransformStamped &msg_AB, 
                             geometry_msgs::TransformStamped &msg_BA,
                             double tparams[]); 
    
    template <typename point_t>
    Eigen::Matrix<double, 6, Eigen::Dynamic> registerCloudTeaserFPFH(pcl::PointCloud<point_t> &source, 
                                                                     pcl::PointCloud<point_t> &target,
                                                                     pcl::PointCloud<point_t> &corrs,
                                                                     tf::StampedTransform &T_AB, 
                                                                     tf::StampedTransform &T_BA,
                                                                     geometry_msgs::TransformStamped &msg_AB, 
                                                                     geometry_msgs::TransformStamped &msg_BA,
                                                                     double tparams[], teaser::FPFHEstimation features );

    //PUBLIC attributes

    double icp_max_corr_dist, icp_max_iter, icp_trns_epsl, icp_ecld_fitn_epsl, icp_ran_rej_thrsh;

};


#endif

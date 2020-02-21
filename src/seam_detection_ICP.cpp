/*
Iterative Closest Point using PCL - Find Rigid Transforms between Pointclouds

Tristan Hill - Weld Seam Detection - Tennessee Technological University

Taken from PCL sample code - 03/02/2018
Modified from more sample code - March 07, 2018
Revisited in February 19, 2020

Robotics Research Group - Mechanical Engineering
*/

// This is going to become seam detection based on Iterative Closest Point - ICP
// It seems to be working - ICP returns a Transformation between the two pointclouds - TH

// make an improved plane filter - 'thick-thin plane filter'

// add a tf broadcaster to show the results of the ICP registration

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <tf/LinearMath/Matrix3x3.h>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Core>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv)
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    ros::init(argc,argv,"seam_detection_ICP");
    ros::NodeHandle node;
    ros::Rate loop_rate(2);

    ros::Publisher pub_lidar = node.advertise<PointCloud> ("/lidar_cloud", 1) ;
    ros::Publisher pub_filtered = node.advertise<PointCloud> ("/filtered_cloud", 1) ;
    ros::Publisher pub_cad = node.advertise<PointCloud> ("/cad_cloud", 1) ;

    // setup a tf for a fram for the result so we we can see it in RVIZ
    static tf::TransformBroadcaster br_result;
    tf::Transform tf_result;




    // read the command line arguments to pick the data file and some other details
    std::string lidar_file = argv[2]; // source cloud
    std::string cad_file = argv[3];   // reference cloud
    double thresh = atof(argv[4]);
    // This path must be changed when I switch workstations - TWH
    //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/seam_detection/images/")+in_file;
    //std::string lidar_cloud_path = std::string("/home/thill/Dropbox/m73_ros/src/seam_detection/images/")+lidar_file;
    //std::string cad_cloud_path = std::string("/home/thill/Dropbox/m73_ros/src/seam_detection/images/")+cad_file;
    std::string lidar_cloud_path = lidar_file;
    std::string cad_cloud_path = cad_file;

    // load the cloud from lidar file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_cloud_path, *lidar_cloud) == -1)
    {
        std::cout<<"Couldn't read image file:"<<lidar_cloud_path;
        return (-1);
    }
    std::cout << "Loaded image file: "<< lidar_cloud_path <<std::endl<<
        lidar_cloud->width * lidar_cloud->height << " Data points from "<< lidar_file << std::endl;

    // load the cloud from CAD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cad_cloud_path, *cad_cloud) == -1)
    {
        std::cout<<"Couldn't read image file:"<<cad_cloud_path;
        return (-1);
    }
    std::cout << "Loaded image file: "<< cad_cloud_path <<std::endl<<
        cad_cloud->width * cad_cloud->height << " Data points from "<< cad_file << std::endl;

    // Filter LIDAR cloud before running ICP
    pcl::PassThrough<pcl::PointXYZ> lpass;
    copyPointCloud(*lidar_cloud,*filtered_cloud);
    lpass.setInputCloud(filtered_cloud);

    float x_min,x_max,y_min,y_max,z_min,z_max;
    x_min=0.0;x_max=0.2;
    y_min=0.1;y_max=0.3;
    z_min=0.026;z_max=0.2;

    lpass.setFilterFieldName ("x");
    lpass.setFilterLimits(x_min,x_max);
    lpass.filter (*filtered_cloud);

    lpass.setFilterFieldName ("y");
    lpass.setFilterLimits(y_min,y_max);
    lpass.filter (*filtered_cloud);

    lpass.setFilterFieldName ("z");
    lpass.setFilterLimits(z_min,z_max);
    lpass.filter (*filtered_cloud);

    std::cout<<"After filtering there are "<<filtered_cloud->width * filtered_cloud->height << " data points in the FILTERED cloud. "<< std::endl;

    // Filter CAD cloud before running ICP - use same xyz limits for now - Why? - I dont think it needs to be filtered
/*
    pcl::PassThrough<pcl::PointXYZ> cpass;
    cpass.setInputCloud(cad_cloud);

    cpass.setFilterFieldName ("x");
    cpass.setFilterLimits(x_min,x_max);
    cpass.filter (*cad_cloud);

    cpass.setFilterFieldName ("y");
    cpass.setFilterLimits(y_min,y_max);
    cpass.filter (*cad_cloud);

    cpass.setFilterFieldName ("z");
    cpass.setFilterLimits(z_min,z_max);
    cpass.filter (*cad_cloud);
*/
  //  std::cout<<"After filtering there are "<<cad_cloud->width * cad_cloud->height << " data points in the CAD cloud. "<< std::endl;
    std::cout<<"There are still "<<cad_cloud->width * cad_cloud->height << " data points in the un-filtered CAD cloud."<< std::endl;

    /*
    // transform the points so we can see a large offset - debug purposes only
    for (size_t i = 0; i < lidar_cloud->points.size (); ++i)
    {
        lidar_cloud->points[i].x = lidar_cloud->points[i].x + 0.0;
        lidar_cloud->points[i].y = lidar_cloud->points[i].y + 0.0;
        lidar_cloud->points[i].z = lidar_cloud->points[i].z + 0.0;
    }
    */

    // perform ICP on the lidar and cad clouds
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(filtered_cloud);
    icp.setInputTarget(cad_cloud);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    Eigen::MatrixXf T_result;
    //Eigen::Matrix3f R_result;





    T_result=icp.getFinalTransformation();
    /*
    R_result<<T_result(0,0),T_result(0,1),T_result(0,2),
              T_result(1,0),T_result(1,1),T_result(1,2),
              T_result(2,0),T_result(2,1),T_result(2,2);
    */


    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    std::cout << "Last Column of T: "<< T_result(0,3)<< std::endl;
    std::cout << "                  "<< T_result(0,3)<< std::endl;
    std::cout << "                  "<< T_result(0,3)<< std::endl;
    std::cout << "                  "<< T_result(0,3)<< std::endl;

    //std::cout << "Just the rotation R: "<< R_result<< std::endl;



    tf::Quaternion q_result;
    //q_result.setRPY(0,0,0);

    tf_result.setOrigin(tf::Vector3(T_result(0,3),T_result(1,3),T_result(2,3)));

    //tf::transformEigenToTF(icp.getFinalTransformation(),tf_result); // this does not compile
    tf::Matrix3x3 R_result(T_result(0,0),T_result(0,1),T_result(0,2),T_result(1,0),T_result(1,1),T_result(1,2),T_result(2,0),T_result(2,1),T_result(2,2));
    R_result.getRotation(q_result);

    // we need to come back to rotation, lets skip it for now.


    tf_result.setRotation(q_result);

    // broadcast the transform to show the result
    br_result.sendTransform(tf::StampedTransform(tf_result,ros::Time::now(),"result","map"));

    // set the fixed reference frame - for now they are all the same
    std::string frame_str="map";
    lidar_cloud->header.frame_id = frame_str;
    filtered_cloud->header.frame_id = frame_str;
    cad_cloud->header.frame_id = frame_str;

    while(ros::ok())
    {
        pub_lidar.publish(lidar_cloud);
        pub_filtered.publish(filtered_cloud);
        pub_cad.publish(cad_cloud);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return (0);
}

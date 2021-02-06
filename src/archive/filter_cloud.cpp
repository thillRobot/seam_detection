/*
    Voxel Fitler for  pointclouds using PCL
    
    Tristan Hill - Weld Seam Detection - Tennessee Technological University
    
    Written - 02/16/2018 

    Robotics Research Group - Mechanical Engineering
*/

#include <iostream>
#include <string>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // define a datatype

int main (int argc, char ** argv)
{
    ros::init(argc,argv,"filter_cloud"); 
    ros::NodeHandle node; 

    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_fromfile", 1) ; 
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_filtered", 1) ;
    ros::Rate loop_rate(2); 

    PointCloud::Ptr cloud_out (new PointCloud);
    PointCloud::Ptr cloud_in (new PointCloud);  

    // read the command line arguments to pick the data file (command line args for entered when launch file is run)
    std::string in_file = argv[1]; 
    std::string out_file = argv[2];

    // This path must be changed when I switch workstations - TWH
    //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/scan2cloud/images/")+test_file;
    std::string in_path = std::string("/home/thill/Dropbox/m73_ros/src/seam_detection/images/")+in_file;
    std::string out_path = std::string("/home/thill/Dropbox/m73_ros/src/seam_detection/images/")+out_file;


    // load the cloud from file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (in_path, *cloud_in) == -1)
    {    
    std::cout<<"Couldn't read image file:"<<in_path;
    return (-1);
    }
   // print the size of the unfiltered cloud
    std::cout << "Loaded image file: "<< in_path <<std::endl<<
            cloud_in->width * cloud_in->height << " Data points from "<< in_file << std::endl;

    // Filter the Cloud
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud_in);
    vox.setLeafSize (0.01f, 0.01f, 0.01f);
    vox.filter (*cloud_out);

    // print the size of the filtered cloud
    std::cout << "The cloud has been filtered and now has " <<
            cloud_out->width * cloud_out->height << " Data points."<< std::endl;
 
    // assign frames to the cloud for tf
    cloud_in->header.frame_id = "map";
    cloud_out->header.frame_id = "map";

    // save the filtered cloud to a .pcd   // SAVING THE OUTPUT IS NOT WORKING!!!
    //pcl::io::savePCDFileASCII ("/home/thill/Dropbox/m73_ros/src/seam_detection/images/test.pcd", *cloud_out);
    pcl::io::savePCDFileASCII (out_path, *cloud_out);
    
    std::cerr << "Saved " << cloud_out->points.size () << " data points to "<< out_file << std::endl;


    // publish both clouds forever
    while(ros::ok())
    {    
        pub1.publish(cloud_in);
        pub2.publish(cloud_out);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

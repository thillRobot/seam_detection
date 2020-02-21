
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

    //ros::Subscriber sub = node.subscribe("/cloud",10, cloud_cb);
    //ros::Publisher pub= node.advertise<sensor_msgs::PointCloud2>("/cloud_out",10);

    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_fromfile", 1) ; 
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_filtered", 1) ;
    ros::Rate loop_rate(2); 

    PointCloud::Ptr cloud_out (new PointCloud);
    PointCloud::Ptr cloud_in (new PointCloud);  

    // read the command line arguments to pick the data file (command line args for entered when launch file is run)
    std::string test_file = argv[1]; 

    // This path must be changed when I switch workstations - TWH
    //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/scan2cloud/images/")+test_file;
    std::string test_path = std::string("/home/thill/Dropbox/m73_ros/src/scan2cloud/images/")+test_file;

    // load the cloud from file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (test_path, *cloud_in) == -1)
    {    
    std::cout<<"Couldn't read image file:"<<test_path;
    return (-1);
    }
    std::cout << "Loaded image file: "<< test_path <<std::endl<<
            cloud_in->width * cloud_in->height << " Data points from "<< test_file << std::endl;

    // Filter the Cloud
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud (cloud_in);
    vox.setLeafSize (0.1f, 0.1f, 0.1f);
    vox.filter (*cloud_out);

    // print the size of the filtered cloud
    std::cout << "The cloud has been filtered and now has " <<
            cloud_out->width * cloud_out->height << " Data points."<< std::endl;
 
    // assign frames to the cloud for tf
    cloud_in->header.frame_id = "map";
    cloud_out->header.frame_id = "map";

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

/*
RANSAC/Segementation based multiple plane detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University

Taken from PCL sample code - 02/14/2018
Updated - 02/17/2018

Robotics Research Group - Mechanical Engineering
*/

#include <iostream>
#include <string>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"RANSAC Plane Detection Node");
    ros::NodeHandle node;
    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_in", 1) ;
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_out1", 1) ;
    ros::Publisher pub3 = node.advertise<PointCloud> ("/cloud_out2", 1) ;

    ros::Rate loop_rate(10);

    PointCloud::Ptr cloud_in (new PointCloud);    //save a copy of the original
    PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy
    PointCloud::Ptr cloud_out1 (new PointCloud);  //these are the clouds for the planes
    PointCloud::Ptr cloud_out2 (new PointCloud);

    // read the command line arguments to pick the data file
    std::string in_file = argv[2];

    // This path must be changed when I switch workstations - TWH
    //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/scan2cloud/images/")+in_file;
    //std::string test_path = std::string("/home/thill/Dropbox/t1600_ros/src/seam_detection/images/")+in_file;
    std::string test_path = in_file;

    // load the cloud from file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (test_path, *cloud_in) == -1)
    {
        std::cout<<"Couldn't read image file:"<<test_path;
        return (-1);
    }
    std::cout << "Loaded image file: "<< test_path <<std::endl<<
        cloud_in->width * cloud_in->height << " Data points from "<< in_file << std::endl;

    // Filter cloud before segementation
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);

    pass.setFilterFieldName ("x");
    pass.setFilterLimits(-0.5,0.5);
    pass.filter (*cloud_in);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits(-1.0,1.0);
    pass.filter (*cloud_in);

    pass.setFilterFieldName ("z");
    pass.setFilterLimits(-0.5,0.5);
    pass.filter (*cloud_in);

    std::cout<<"After filtering there are "<<cloud_in->width * cloud_in->height << " data points. "<< std::endl;

    pcl::copyPointCloud(*cloud_in,*cloud);
    //cloud=cloud_in;
    //double max_distance=0.2;
    double max_distance = atof(argv[3]);

    // All the objects needed
    pcl::PCDReader reader;
    //pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
    //reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
      std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
      std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
      writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }

    /*
    // Get segmentation ready
    pcl::ModelCoefficients::Ptr coeffs1(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coeffs2(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(max_distance);

    // Fit the first plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeffs1);

    // Extract inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*cloud_out1);

    // Extract outliers to be used to find second plane
    extract.setNegative(true);
    extract.filter(*cloud);

    //Eigen::Vector3f axis(1,0,0); // This doesnt seem to matter ...
    //seg.setAxis(axis);

    // Show the results of the first segmentation
    std::cout<<"After first segmentation there are "<<cloud_out1->width * cloud_out1->height << " data points in the first plane. "<< std::endl;
    std::cout<<"and there are "<<cloud->width * cloud->height << " data points not in the first plane. "<< std::endl;
    std::cout<<"The coefficients of the first plane are: "<<std::endl;
    std::cout<<"A: "<<coeffs1->values[0]<<std::endl;
    std::cout<<"B: "<<coeffs1->values[1]<<std::endl;
    std::cout<<"C: "<<coeffs1->values[2]<<std::endl;
    std::cout<<"D: "<<coeffs1->values[3]<<std::endl;

    // Fit a second plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeffs2);

    // Extract inliers
    extract.setInputCloud(cloud);
    extract.setNegative(false);
    extract.setIndices(inliers);
    extract.filter(*cloud_out2);

    // Extract outliers leftover
    extract.setNegative(true);
    extract.filter(*cloud);

    // Show the results of the second segmentation
    std::cout<<"After second segmentation there are "<<cloud_out2->width * cloud_out2->height << " data points in the first plane. "<< std::endl;
    std::cout<<"and there are "<<cloud->width * cloud->height << " data points not in the second plane. "<< std::endl;
    std::cout<<"The coefficients of the second plane are: "<<std::endl;
    std::cout<<"A: "<<coeffs2->values[0]<<std::endl;
    std::cout<<"B: "<<coeffs2->values[1]<<std::endl;
    std::cout<<"C: "<<coeffs2->values[2]<<std::endl;
    std::cout<<"D: "<<coeffs2->values[3]<<std::endl;

    */
    pcl::copyPointCloud(*cloud_cylinder,*cloud_out1);

    cloud_in->header.frame_id = "map";
    cloud_out1->header.frame_id = "map";
    cloud_out2->header.frame_id = "map";
    //publish forever
    while(ros::ok())
    {
        pub1.publish(cloud_in);
        pub2.publish(cloud_out1);
        pub3.publish(cloud_out2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

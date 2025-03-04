/*
RANSAC/Segementation based multiple plane/line/point detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University 

Taken from PCL sample code - 02/14/2018 
Updated - 02/17/2018 - 02/20/2018  - added solutions for intersection of planes
Updated - 02/26/2018  - adding 'visualization markers' for the planes - from the example code online

Updated - 03/07/2018 - adding lots of stuff, docs are out of date

Updated - 03/016/2018 - make an improved plane filter - 'thick-thin plane filter'


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

#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"seam_detection_RANSAC"); 
    ros::NodeHandle node; 
    ros::Publisher pub0 = node.advertise<PointCloud> ("/cloud_in", 1) ;
    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_out1", 1) ;
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_out2", 1) ;
    ros::Publisher pub3 = node.advertise<PointCloud> ("/cloud_out3", 1) ;
    
    ros::Rate loop_rate(2); 

    PointCloud::Ptr cloud_in (new PointCloud);    //save a copy of the original
    PointCloud::Ptr cloud_out1 (new PointCloud);  //these are the clouds for the planes
    PointCloud::Ptr cloud_out2 (new PointCloud);
    PointCloud::Ptr cloud_out3 (new PointCloud);    
    PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy
    PointCloud::Ptr cloud_plane (new PointCloud);

    geometry_msgs::PointStamped point_out;

    // read the command line arguments to pick the data file and some other details 
    std::string in_file = argv[2]; 
    double thresh1 = atof(argv[3]);
    double thresh2 = atof(argv[4]);

    // This path must be changed when I switch workstations - TWH
    // std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/seam_detection/images/")+in_file;
    std::string test_path = std::string("/home/thill/Dropbox/m73_ros/src/seam_detection/images/")+in_file;

    // load the cloud from file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (test_path, *cloud_in) == -1)
    {    
        std::cout<<"Couldn't read image file:"<<test_path;
        return (-1);
    }
    std::cout << "Loaded image file: "<< test_path <<std::endl<<
        cloud_in->width * cloud_in->height << " Data points from "<< in_file << std::endl;

    // Filter LIDAR cloud before running ICP
    pcl::PassThrough<pcl::PointXYZ> lpass;
    lpass.setInputCloud(cloud_in);
    
    float x_min,x_max,y_min,y_max,z_min,z_max;
    x_min=0.1;x_max=0.6;
    y_min=0.1;y_max=0.8;
    z_min=-0.5;z_max=0.2;

    lpass.setFilterFieldName ("x");
    lpass.setFilterLimits(x_min,x_max);
    lpass.filter (*cloud_in);
    
    lpass.setFilterFieldName ("y");
    lpass.setFilterLimits(y_min,y_max);
    lpass.filter (*cloud_in);
        
    lpass.setFilterFieldName ("z");
    lpass.setFilterLimits(z_min,z_max);
    lpass.filter (*cloud_in);
        
    std::cout<<"After filtering there are "<<cloud_in->width * cloud_in->height << " data points. "<< std::endl;
      
    // Get segmentation ready
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coeffs1(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coeffs2(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coeffs3(new pcl::ModelCoefficients);
     
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    

    pcl::copyPointCloud(*cloud_in,*cloud);
    
    int k=0;
    while(k<3)
    {
        // Segment the first plane
        // set the threshold - larger 
        seg.setDistanceThreshold(thresh1);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coeffs);

        // Extract inliers
        extract.setNegative(false);    
        extract.setIndices(inliers);

        extract.setInputCloud(cloud);
        extract.filter(*cloud_plane);
        
        // Extract outliers to be used to find the next plane (not the smaller plane of this plane!)
        extract.setNegative(true);
        extract.filter(*cloud);
    
        // Segment the second plane
        // set the threshold - smaller 
        seg.setDistanceThreshold(thresh2);
        seg.setInputCloud(cloud_plane);
        seg.segment(*inliers, *coeffs);           
         
        extract.setNegative(false);  
        extract.setIndices(inliers);
              
        extract.setInputCloud(cloud_plane);
        extract.filter(*cloud_plane);    

        //Eigen::Vector3f axis(1,0,0); // This doesnt seem to matter ...
        //seg.setAxis(axis);
        
        // Show the results of the first and second segmentation   
        std::cout<<"After segmentation there are "<<cloud_plane->width * cloud_plane->height << " data points in the plane. "<< std::endl;
        std::cout<<"and there are "<<cloud->width * cloud->height << " data points not in the plane. "<< std::endl;
        std::cout<<"The coefficients of the plane are: "<<std::endl;
        std::cout<<"A: "<<coeffs->values[0]<<std::endl;
        std::cout<<"B: "<<coeffs->values[1]<<std::endl;
        std::cout<<"C: "<<coeffs->values[2]<<std::endl;
        std::cout<<"D: "<<coeffs->values[3]<<std::endl;
        
        if(k==0)
        {
            pcl::copyPointCloud(*cloud_plane,*cloud_out1);
            coeffs1->values=coeffs->values;
        }
        else if(k==1)    
        {
            pcl::copyPointCloud(*cloud_plane,*cloud_out2);
            coeffs2->values=coeffs->values;
        }            
        else 
        {
            pcl::copyPointCloud(*cloud_plane,*cloud_out3); 
            coeffs3->values=coeffs->values;        
        }
  
        k++;
  
    }
    

  
    // set the fixed reference frame - for now they are all the same
    std::string frame_str="map";
    
    cloud_in->header.frame_id = frame_str;
    cloud_out1->header.frame_id = frame_str;
    cloud_out2->header.frame_id = frame_str;
    cloud_out3->header.frame_id = frame_str;

    //publish forever
    while(ros::ok())
    {    
        pub0.publish(cloud_in);
        pub1.publish(cloud_out1);
        pub2.publish(cloud_out2);
        pub3.publish(cloud_out3);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


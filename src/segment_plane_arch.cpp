/*
RANSAC/Segementation based plane detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University 

Taken from PCL sample code - 02/14/2018 

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

#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"RANSAC Plane Detection Node"); 
    ros::NodeHandle node; 
    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_in", 1) ;
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_out1", 1) ;
    ros::Publisher pub3 = node.advertise<PointCloud> ("/cloud_out2", 1) ;
    //ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_leftover", 1) ;

    ros::Rate loop_rate(10); 

    PointCloud::Ptr cloud_in (new PointCloud);
    PointCloud::Ptr cloud_out1 (new PointCloud);
    PointCloud::Ptr cloud_out2 (new PointCloud);    
  
    // read the command line arguments to pick the data file
    std::string in_file = argv[2]; 

    // This path must be changed when I switch workstations - TWH
    //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/scan2cloud/images/")+in_file;
    std::string test_path = std::string("/home/thill/Dropbox/m73_ros/src/scan2cloud/images/")+in_file;

    // load the cloud from file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (test_path, *cloud_in) == -1)
    {    
        std::cout<<"Couldn't read image file:"<<test_path;
        return (-1);
    }
    std::cout << "Loaded image file: "<< test_path <<std::endl<<
        cloud_in->width * cloud_in->height << " Data points from "<< in_file << std::endl;

    //double max_distance=0.2;
    double max_distance = atof(argv[3]);

    // Get segmentation ready
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(max_distance);

    // Fit a plane
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coeffs);

    // Extract inliers
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);    
    extract.filter(*cloud_out1);

    // Extract outliers
    extract.setNegative(true);
    extract.filter(*cloud_out2);
    
    //cloud_in->swap(*cloud_out);
    


    // Fit a plane
    //seg.setInputCloud(cloud_in);
    //seg.segment(*inliers, *coefficients);

    // Extract inliers
    //extract.setInputCloud(cloud_in);
    //extract.setIndices(inliers);
    //extract.setNegative(true);
    //pcl::PointCloud<pcl::PointXYZ> cloudF;    
    //extract.filter(*cloud_out2);


    //cloud_in->swap(*cloud_out);
   
    std::cout<<"Before segmentation there were "<<cloud_in->width * cloud_in->height << " data points. "<< std::endl;
    std::cout<<"After segmentation1 there are "<<cloud_out1->width * cloud_out1->height << " data points in the first plane. "<< std::endl;
    std::cout<<"After segmentation2 there are "<<cloud_out2->width * cloud_out2->height << " data points not in the first plane. "<< std::endl;
    std::cout<<"The coefficients of the first plane are: "<<std::endl;
    std::cout<<"A: "<<coeffs->values[0]<<std::endl;
    std::cout<<"B: "<<coeffs->values[1]<<std::endl;
    std::cout<<"C: "<<coeffs->values[2]<<std::endl;
    std::cout<<"D: "<<coeffs->values[3]<<std::endl;

   // for (std::vector<int>::const_iterator i = coeffs->begin(); i != coeffs->end(); ++i)
   //     std::cout << *i << ' ';

    //std::cout<<   

    cloud_in->header.frame_id = "map";
    cloud_out1->header.frame_id = "map";
    cloud_out2->header.frame_id = "map";
    

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



    //std::vector<int> inliers;
    //pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // instantiate RandomSampleConsensus object and compute the appropriated model
    //pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

    //pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);

    //ransac.setDistanceThreshold (.01);
    //ransac.computeModel();
    //ransac.getInliers(inliers);

    //std::vector< int > model;
    //ransac.getModel(model);  

    //std::cout<<"The model VectorXf: "<<std::endl ;
    //for (std::vector<int>::const_iterator i = model.begin(); i != model.end(); ++i)
    //    std::cout << *i << ' ';
    //std::cout<<std::endl;
    //std::cout<<"The model is:"<< std::string(model) << std::endl;

    //pcl::SacModel model_type; 	
    //model_type=model_p.getModelType();
    //std::cout<<"The model type is:"<< model_type << std::endl;

    //Eigen::VectorXf coefs;
    //ransac.getModelCoefficients(coefs);
    //std::cout<<"The coefficients of the model:"<<std::endl<< coefs << std::endl;

    // copy all inliers of the model computed to the PointCloud for to publish
    //pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_pub);

    
    //cloud_pub2->header.frame_id = "map";
    //cloud_pub->height = cloud_pub->width = 1;
    //pcl_conversions::fromPCL(cloud_ransac, cloud_pub); 

    // find a second plane
    // start by taking out the inliers from the first plane extraction

    /*
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Extract inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ> cloud_leftover;
    extract.filter(cloud_leftover);
    cloud->swap(cloud_leftover);

    cloud_leftover->header.frame_id = "map";
    */
    /*
    cloud_pub->header.frame_id = "map";

    while(ros::ok())
    {    
        pub.publish(cloud_pub);
        //pub2.publish(cloud_pub2);
        ros::spinOnce();
        loop_rate.sleep();
    }
    */
    /*
    // Uncomment this block if you want to use the '3d Viewer' to see the clouds
    // creates the visualization object and adds either our orignial cloud or all of the inliers
    // depending on the command line arguments specified.

    // copy all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (pcl::console::find_argument (argc, argv, "-f") >= 0)
    viewer = simpleVis(cloud_final);
    else
    viewer = simpleVis(cloud);
    while (!viewer->wasStopped ())
    {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    */



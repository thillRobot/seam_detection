
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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <aubo_control/LidarState.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <pcl/filters/radius_outlier_removal.h>
#include <scan2cloud/lidarScanState.h>

ros::ServiceClient lidar_state_client;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // define a datatype

ros::Publisher pub;

PointCloud::Ptr cloud (new PointCloud);
PointCloud::Ptr cloud_out (new PointCloud);

bool start_lidar_scan=false;
bool debouced_once=false;
scan2cloud::lidarScanState lidar_state_srv;

void lidar_state(const std_msgs::Bool::ConstPtr &msg)
{

  // store request
    start_lidar_scan=msg->data; // recieve command

    // turn on/off lidar		
    lidar_state_srv.request.start_lidar_scanning=start_lidar_scan; // fill in request
    lidar_state_client.call(lidar_state_srv);  // call the service maybe put a delay at start of call server
    ROS_INFO("LIDAR ON");

   if(!start_lidar_scan){
        // reset debounce
        debouced_once=false;
    }

}



void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg_in)
{

    PointCloud::Ptr cloud_in (new PointCloud); 
    PointCloud::Ptr cloud_temp (new PointCloud);  
    PointCloud::Ptr cloud_test (new PointCloud);      

    if(start_lidar_scan && !debouced_once){
        // clear point cloud

        ROS_INFO("CLEARING POINT CLOUD");
        cloud_out->clear();

        pub.publish(*cloud_out);
        ros::spinOnce();

        debouced_once=true;
    }


    pcl::fromROSMsg(*cloud_msg_in,*cloud_in);

    //std::cout<<cloud_in->width*cloud_in->height<<"Data points in."<<std::endl;

    cloud=cloud_in;
    *cloud_in=*cloud_out+*cloud;
     
    //std::cout<<cloud_in->width*cloud_in->height<<"Data points out after concatenation."<<std::endl;     
   
    

    pcl::PassThrough<pcl::PointXYZ> pass;

    cloud_in->header.frame_id="base_Link";

    pass.setInputCloud(cloud_in);
/*
    // GLOBAL FRAME LIMITS
    pass.setFilterFieldName ("x");
    //pass.setFilterLimits(-0.8,0.8);
    pass.setFilterLimits(-1.5,1.5);
    pass.filter (*cloud_in);

    pass.setFilterFieldName ("y");
    //pass.setFilterLimits(-1.3, 0);
    pass.setFilterLimits(-1.0,-0.25);
    pass.filter (*cloud_in);
        
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(-0.1,0.3);
    pass.filter (*cloud_in);
*/
    pcl::copyPointCloud(*cloud_in,*cloud_out);

}   

int main (int argc, char ** argv)
{
    ros::init(argc,argv,"merge_cloud"); 
    ros::NodeHandle node; 
    ros::Subscriber sub = node.subscribe("/cloud",10, cloud_cb);
    ros::Subscriber lidar_state_sub = node.subscribe("/cr_weld/scan_state",10, lidar_state);
    //ros::Publisher publicador= ns.advertise<sensor_msgs::PointCloud2>("/cloud_out",10);
    pub = node.advertise<PointCloud> ("/cloud_out", 10) ;

    lidar_state_client = node.serviceClient<scan2cloud::lidarScanState>("lidar_scan_state");


    ros::Rate loop_rate(15); 
    ros::spinOnce();
    
    cloud_out->header.frame_id="base_Link";

    while(ros::ok())
    {       
        
        pub.publish(*cloud_out);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //ROS_INFO("Fin suscriptor");
    return 0;
}
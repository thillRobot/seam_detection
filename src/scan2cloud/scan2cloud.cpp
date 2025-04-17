#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <scan2cloud/lidarScanState.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <aubo_control/LidarState.h>

ros::Publisher lidar_state_pub;

aubo_control::LidarState lidar_status;
bool start_lidar_scan=false;
bool pause_lidar_scan=false;

bool lidar_state(scan2cloud::lidarScanState::Request  &req, scan2cloud::lidarScanState::Response &res)
{

  // store request
  start_lidar_scan=req.start_lidar_scanning; // recieve command
  pause_lidar_scan=req.pause_lidar_scanning;
	res.start_command_recieved=true; // respond to the client

	return true;	
}

ros::Publisher scan_pub_;
class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "scan_filtered", 1000),
    laser_notifier_(laser_sub_,listener_, "base_Link", 1000)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.05)); //laser_notifier_.setTolerance(ros::Duration(0.2)); 
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud",1);
  }


  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

if(start_lidar_scan && !pause_lidar_scan){
//if(1){

	
        //ROS_INFO("****************** Scan Projection Callback **************");
        sensor_msgs::PointCloud2 cloud;
        try
        {
			


            projector_.transformLaserScanToPointCloud("base_Link", *scan_in, cloud,listener_);

			
            //ROS_INFO("****************** Performing Projection **************");
        }
        catch (tf::TransformException& e)
        {
          ROS_INFO("TRANSFORM ERROR in scan2cloud.cpp");
            //std::cout << e.what();
            //ROS_INFO("****************** Projection Error **************");
            return;
        }
        
        // Do something with cloud.

        scan_pub_.publish(cloud);

      }

      lidar_status.header.stamp=ros::Time::now();
      lidar_status.lidarState=start_lidar_scan;
      lidar_state_pub.publish(lidar_status);
  }



}; // semi colon after class def

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "scan2cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  lidar_state_pub = n.advertise<aubo_control::LidarState>("lidar_state", 1);
  ros::ServiceServer service = n.advertiseService("lidar_scan_state", lidar_state);

  // wait for lidar to boot up 
  ros::Duration(5).sleep();
 
  ros::spin();
  
  return 0;
}

#include <ros/ros.h>
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
#include <pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/String.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <scan2cloud/pcdFileCmd.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // define a datatype



// recieves a basic string cmd


// recieves a full cmd
/*
Header header
string file_name
float32[6] transformation # x,y,z,yaw,pitch,roll. meters and degrees.
*/

std::string pcd_file_name;
int new_file=false;
double pcd_offset_x=0; 
double pcd_offset_y=0; 
double pcd_offset_z=0; 
double pcd_offset_Yaw=0; 
double pcd_offset_Pitch=0; 
double pcd_offset_Roll=0; 
 
 
bool keep_prev=false;
void fileCmdCallback(const scan2cloud::pcdFileCmd::ConstPtr &msg){

    keep_prev=msg->overlay;

    pcd_file_name=msg->file_name;
    pcd_offset_x=msg->transformation[0];
    pcd_offset_y=msg->transformation[1];
    pcd_offset_z=msg->transformation[2];
    pcd_offset_Yaw=msg->transformation[3];
    pcd_offset_Pitch=msg->transformation[4];
    pcd_offset_Roll=msg->transformation[5];

    new_file=true;


}

int main (int argc, char ** argv)
{
    ros::init(argc,argv,"publish_pcd"); 
    ros::NodeHandle n;  
    ros::Publisher pub = n.advertise<PointCloud> ("/cloud_out", 1, true) ;
    ros::Subscriber file_cmd_sub = n.subscribe("/pcd_file_cmd", 1 , fileCmdCallback);

    ros::Rate loop_rate(20); 

        
    // read in pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev (new pcl::PointCloud<pcl::PointXYZ>);


    new_file=false;

    while(ros::ok())
    {       

        if(new_file){

            cloud_out->clear();
            std::string file_path=( std::string(getenv("HOME"))+"/cr_weld_ws/src/penn_state_core/" + pcd_file_name + ".pcd").c_str() ;

            if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud_out) == -1) //* load the file

            {
                PCL_ERROR ("Couldn't read PCD file \n");
                return (-1);
            }


            cloud_out->header.frame_id="base_Link"; // must apply a header before publishing


            Eigen::Matrix4f transform;

                // convert euler YPR to a 3x3 rotation matrix
            Eigen::Vector3d euler;
            double deg_to_rad= M_PI/180;
            euler << pcd_offset_Yaw*deg_to_rad, pcd_offset_Pitch*deg_to_rad, pcd_offset_Roll*deg_to_rad;
            
            double yaw=euler(0);
            double pitch=euler(1);
            double roll=euler(2);
            Eigen::Matrix3d Rx, Ry, Rz, R;
            Rx << 1,        0,          0,
                    0, cos(roll), -sin(roll),
                    0, sin(roll),  cos(roll);

            Ry <<      cos(pitch), 0, sin(pitch), 
                        0,       1,      0,
                    -sin(pitch),  0, cos(pitch);

            Rz <<      cos(yaw),  -sin(yaw),    0,
                        sin(yaw),  cos(yaw),     0,
                        0,               0,      1;

            R=Rz*Ry*Rx;


            transform << R(0,0), R(0,1), R(0,2), pcd_offset_x,
                        R(1,0), R(1,1), R(1,2), pcd_offset_y,
                        R(2,0), R(2,1), R(2,2), pcd_offset_z,
                        0,      0,      0,      1;


            pcl::transformPointCloud (*cloud_out, *cloud_out, transform);
            new_file=false;

            
        }

        pub.publish(*cloud_out);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    

 

    return 0;
}


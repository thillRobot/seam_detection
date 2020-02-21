/*
RANSAC/Segementation based multiple plane/line/point detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University

Taken from PCL sample code - 02/14/2018
Updated - 02/17/2018 - 02/20/2018

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


#include <Eigen/Dense>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"RANSAC Plane Detection Node");
    ros::NodeHandle node;
    ros::Publisher pub0 = node.advertise<PointCloud> ("/cloud_in", 1) ;
    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_out1", 1) ;
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_out2", 1) ;
    ros::Publisher pub3 = node.advertise<PointCloud> ("/cloud_out3", 1) ;

    ros::Publisher pub4 = node.advertise<geometry_msgs::PointStamped> ("/point_out", 1) ;
    ros::Publisher pub5 = node.advertise<PointCloud> ("/cloud_line", 1) ;

    ros::Rate loop_rate(2);

    PointCloud::Ptr cloud_in (new PointCloud);    //save a copy of the original
    PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy
    PointCloud::Ptr cloud_plane (new PointCloud);
    PointCloud::Ptr cloud_out1 (new PointCloud);  //these are the clouds for the planes
    PointCloud::Ptr cloud_out2 (new PointCloud);
    PointCloud::Ptr cloud_out3 (new PointCloud);

    PointCloud::Ptr cloud_line (new PointCloud);

    geometry_msgs::PointStamped point_out;

    // read the command line arguments to pick the data file and some other details
    std::string in_file = argv[2];
    double max_distance = atof(argv[3]);

    // This path must be changed when I switch workstations - TWH
    //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/seam_detection/images/")+in_file;
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
    seg.setDistanceThreshold(max_distance);

    pcl::copyPointCloud(*cloud_in,*cloud);

    int k=0;
    while(k<3)
    {
        // Fit the first plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coeffs);

        // Extract inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);

        extract.setNegative(false);
        extract.filter(*cloud_plane);

        // Extract outliers to be used to find the next plane
        extract.setNegative(true);
        extract.filter(*cloud);

        //Eigen::Vector3f axis(1,0,0); // This doesnt seem to matter ...
        //seg.setAxis(axis);

        // Show the results of the first segmentation
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

    // Do some fancy math with the Eigen Class
    // first map elements to SC's naming

    float A1,B1,C1,D1,A2,B2,C2,D2,A3,B3,C3,D3;

    A1=coeffs1->values[0];B1=coeffs1->values[1];C1=coeffs1->values[2];D1=coeffs1->values[3];
    A2=coeffs2->values[0];B2=coeffs2->values[1];C2=coeffs2->values[2];D2=coeffs2->values[3];
    A3=coeffs3->values[0];B3=coeffs3->values[1];C3=coeffs3->values[2];D3=coeffs3->values[3];

    // Solve for the point that is the intersection of the three planes
    Eigen::Matrix3f A_p;
    Eigen::Vector3f b_p,x_p;

    A_p << A1,B1,C1,
         A2,B2,C2,
         A3,B3,C3;

    b_p << D1,D2,D3;

    x_p = A_p.colPivHouseholderQr().solve(b_p); // There are several solvers to choose from

    std::cout << "The matrix A is:\n" << A_p << std::endl;
    std::cout << "The vector b is:\n" << b_p << std::endl;
    std::cout << "The solution x is:\n" << x_p << std::endl;

    point_out.point.x=-x_p(0);
    point_out.point.y=-x_p(1);
    point_out.point.z=-x_p(2);

    // Solve for the line that intersect the first and second planes
    Eigen::Vector3f L_vec;
    Eigen::Vector3f L;

    Eigen::MatrixXf A_l(2,2);
    Eigen::VectorXf b_l(2);
    Eigen::VectorXf x_l(2);
    Eigen::VectorXf P_l(3);

    L_vec << B1*C2-C1*B2 , C1*A2-A1*C2 , A1*B2-B1*A2;

    float L_mag,X_mag,Y_mag,Z_mag;
    X_mag=std::pow(L_vec(0),2);
    Y_mag=std::pow(L_vec(1),2);
    Z_mag=std::pow(L_vec(2),2);
    L=L_vec/(std::pow((X_mag+Y_mag+Z_mag),0.5));

    std::cout << "L_vec is:\n" << L_vec << std::endl;
    std::cout << "L is:\n" << L << std::endl;

    if(L_vec(1)==0)
    {
        std::cout<<"solution error"<<std::endl;
    }
    else
    {
        std::cout<<"solution exists"<<std::endl;

        // setup the matrix system
        A_l << A1,B1,
                 A2,B2;
        b_l << -D1,-D2;

        x_l=A_l.inverse()*b_l;

        std::cout << "The solution x is:\n" << x_l << std::endl;

        P_l<<x_l(0),x_l(1),0;

        // setup a cloud to put the line in
        cloud_line->width    = 50;
        cloud_line->height   = 1;
        cloud_line->is_dense = false;
        cloud_line->points.resize (cloud_line->width * cloud_line->height);

        float s;
        float s_inc=0.01;

        // solve for the starting parameter from the intersection of the three planes
        s=(-x_p(0)-P_l(0))/L(0);

        // populate the cloud with data using the parametric equation of the line
        for (size_t i = 0; i < cloud_line->points.size (); ++i)
        {
            //equations of the line
            cloud_line->points[i].x = P_l(0)+L(0)*s;
            cloud_line->points[i].y = P_l(1)+L(1)*s;
            cloud_line->points[i].z = P_l(2)+L(2)*s;
            //increment the paramter
            s=s+s_inc;
        }

    }

    // set the reference frames
    cloud_line->header.frame_id="map";
    point_out.header.frame_id="map";
    cloud_in->header.frame_id = "map";
    cloud_out1->header.frame_id = "map";
    cloud_out2->header.frame_id = "map";
    cloud_out3->header.frame_id = "map";

    //publish forever
    while(ros::ok())
    {
        pub0.publish(cloud_in);
        pub1.publish(cloud_out1);
        pub2.publish(cloud_out2);
        pub3.publish(cloud_out3);

        pub4.publish(point_out);
        pub5.publish(cloud_line);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

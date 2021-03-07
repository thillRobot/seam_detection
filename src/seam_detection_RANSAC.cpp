/*
RANSAC/Segementation based multiple plane/line/point detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University

Taken from PCL sample code - 02/14/2018
Updated - 02/17/2018 - 02/20/2018  - added solutions for intersection of planes
Updated - 02/26/2018  - adding 'visualization markers' for the planes - from the example code online

Updated - 03/07/2018 - adding lots of stuff, docs are out of date

Updated - 03/20/2018 - double filter, aka thick-thin filter is working with scene1
                     - finished the math for using the ros -vis markers

Updated - 03/22/2018 - add analysis data file output

Revisted - 02/22/2020 - I have no idea what I am doing


Robotics Research Group - Mechanical Engineering
*/

#include <iostream>
#include <fstream>
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
#include <math.h>

#include <tf/transform_datatypes.h>
//#include <Quaternion.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{

    std::cout<<"*************************************************************"<<endl;
    std::cout<<"************ Seam Detection RANSAC v1.4 *********************"<<endl;
    std::cout<<"*************************************************************"<<endl;
    std::cout<<"Using PCL version:"<< PCL_VERSION_PRETTY <<endl<<endl;

    ros::init(argc,argv,"seam_detection_RANSAC");
    ros::NodeHandle node;

    //pubs for the pointclouds - planes
    ros::Publisher pub0 = node.advertise<PointCloud> ("/cloud_in", 1) ;
    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_out1", 1) ;
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_out2", 1) ;
    ros::Publisher pub3 = node.advertise<PointCloud> ("/cloud_out3", 1) ;

    //pubs for the point and line
    ros::Publisher pub4 = node.advertise<geometry_msgs::PointStamped> ("/point_out", 1) ;
    ros::Publisher pub5 = node.advertise<PointCloud> ("/cloud_line", 1) ;

    //pubs for the plane markers
    ros::Publisher pub6 = node.advertise<visualization_msgs::Marker>("/plane_marker1", 1);
    ros::Publisher pub7 = node.advertise<visualization_msgs::Marker>("/plane_marker2", 1);
    ros::Publisher pub8 = node.advertise<visualization_msgs::Marker>("/plane_marker3", 1);

    ros::Rate loop_rate(2);

    // instatiate the objects for.. everything!
    PointCloud::Ptr cloud_in (new PointCloud);    //save a copy of the original
    PointCloud::Ptr cloud (new PointCloud);       //use this as the working copy
    PointCloud::Ptr cloud_plane (new PointCloud);
    PointCloud::Ptr cloud_out1 (new PointCloud);  //these are the clouds for the planes
    PointCloud::Ptr cloud_out2 (new PointCloud);
    PointCloud::Ptr cloud_out3 (new PointCloud);
    PointCloud::Ptr cloud_line (new PointCloud);

    geometry_msgs::PointStamped point_out;

    visualization_msgs::Marker marker1; // notice the markers are not pointers
    visualization_msgs::Marker marker2;
    visualization_msgs::Marker marker3;

    visualization_msgs::Marker marker; // this is a template for the 3 markers

    // set the fixed reference frame - for basically everything
    std::string frame_str="map";

    // read the command line arguments to pick the data file and some other details
    std::string in_file = argv[2];
    std::string analysis_file = argv[3];
    double thresh1 = atof(argv[4]);
    double thresh2 = atof(argv[5]);

    // This path must be changed when I switch workstations - TWH
    // std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/seam_detection/images/")+in_file;
    std::string test_path = in_file;
    std::string analysis_path = analysis_file;

    // load the cloud from file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (test_path, *cloud_in) == -1)
    {
        std::cout<<"Couldn't read image file:"<<test_path;
        return (-1);
    }
    std::cout << "Loaded image file: "<< test_path <<std::endl<<
        cloud_in->width * cloud_in->height << " Data points from "<< in_file << std::endl;

    //setup a data file to dump the analysis

    //ofstream data_file;
    //data_file.open("scene1_analysis.txt");
    //data_file<<"Seam Detection Analysis Data - Scene1\n";
    //data_file.close();

    std::cout<<"About to create data file."<<std::endl;
    std::ofstream myfile;
    myfile.open(analysis_path.c_str());
    std::cout<<"Now check if it is open."<<std::endl;
    //if (myfile.good())
    //{
    //    myfile << "The file is good!\n";
    //    std::cout<<"The file seems to be good."<<std::endl;
    //}
    if (myfile.is_open())
    {
        myfile << "Seam Detection Data Analysis - Scene1 - TTU"<<std::endl;
    }
    else std::cout << "Unable to open file";
    //return 0;
    std::cout<<"Finished creating data file."<<std::endl;

    bool is_new=1;
    if(is_new) // what is this? TH
    {
        myfile<<"A1i"<<","<<"B1i"<<","<<"C1i"<<","<<"A2i"<<","<<"B2i"<<","<<"C2i"<<","<<"A3i"<<","<<"B3i"<<","<<"C3i"<<std::endl;
    }

    // Filter LIDAR cloud before running using RANSAC
    pcl::PassThrough<pcl::PointXYZ> lpass;
    lpass.setInputCloud(cloud_in);

    float x_min,x_max,y_min,y_max,z_min,z_max;
    x_min=-0.1;x_max=0.50;
    y_min=0.4;y_max=0.8;
    z_min=-0.2;z_max=0.2;

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
        cloud_line->width    = 250;
        cloud_line->height   = 1;
        cloud_line->is_dense = false;
        cloud_line->points.resize (cloud_line->width * cloud_line->height);

        float s;
        float s_inc=-0.001;

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

    float px,py,pz;
    float Xn, Zn;
    float x1,x2,x3,y1,y2,y3,z1,z2,z3;
    float Z1,Z2,Z3,Z4,X1,X2,X3,Y1,Y2,Y3;
    float roll,pitch,yaw,alpha,beta,gamma;

    int j=0;
    while (j<3)
    {
        if (j==0) // choose the plane solution to use
        {
            Z1=A1;  // this shows the plane 1 (red)   (Why are planes 2 and 2 switched!!!)
            Z2=B1;  // with - gamma,beta,alpha
            Z3=C1;
            Z4=C1;
        }
        else if(j==1)
        {
            Z1=A2; // this shows plane 2 (green)
            Z2=B2; // with - gamma,beta,alpha
            Z3=C2;
            Z4=D2;
        }
        else
        {
            Z1=A3;  // this shows plane 3 (blue)
            Z2=B3;  // with - gamma,beta,alpha
            Z3=C3;
            Z4=D3;
        }

        X1=1; // non-normalized x vec of plane (local frame) - this is a free guess
        X2=0;
        X3=(-X1*Z1-X2*Z2)/Z3; // solve for the 3rd component of x with (x dot z = 0)

        // normalize the x and z vectors
        Xn=sqrt(pow(X1,2)+pow(X2,2)+pow(X3,2));
        Zn=sqrt(pow(Z1,2)+pow(Z2,2)+pow(Z3,2));
        x1=X1/Xn; x2=X2/Xn; x3=X3/Xn;
        z1=Z1/Zn; z2=Z2/Zn; z3=Z3/Zn;

        // cross z and x to get y
        y1=z2*x3-z3*x2;
        y2=-(z1*x3-z3*x1);
        y3=z1*x2-z2*x1;

        // show the rotation matrix formed by the unit vectors
        Eigen::Matrix3f R;
        R<< x1,y1,z1,
            x2,y2,z2,
            x3,y3,z3;

        std::cout << "The Rotation Matrix R is:\n" << R << std::endl;

        // these equations come from (http://planning.cs.uiuc.edu/node103.html) - looks legit :)
        // and here - (https://robotics.stackexchange.com/questions/8516/getting-pitch-yaw-and-roll-from-rotation-matrix-in-dh-parameter?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa)
        alpha= atan2(x2,x1); //x - yaw
        beta = atan2(-x3,sqrt( pow(y3,2)+pow(z3,2))); //y - pitch
        gamma= atan2(y3,z3); //z - roll

        std::cout << "Alpha: "<< alpha << std::endl;
        std::cout << "Beta: "<< beta << std::endl;
        std::cout << "Gamma: "<< gamma << std::endl;

        //map alpha beta gamma to roll pitch yaw
        roll=gamma;          //z
        pitch=beta;          //y
        yaw=alpha;           //x

        std::cout << "Roll: "<< roll << std::endl;
        std::cout << "Pitch: "<< pitch << std::endl;
        std::cout << "Yaw: "<< yaw << std::endl;

        // convert from RPY to Quaternions - (from wikipedia)
  	    double cr = cos(roll * 0.5);
  	    double sr = sin(roll * 0.5);
  	    double cp = cos(pitch * 0.5);
  	    double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
  	    double sy = sin(yaw * 0.5);

        double qx,qy,qz,qw;
        qw = cy * cr * cp + sy * sr * sp;
  	    qx = cy * sr * cp - sy * cr * sp;
  	    qy = cy * cr * sp + sy * sr * cp;
  	    qz = sy * cr * cp - cy * sr * sp;

        // add a 'visualization marker' for the planes - should decrease msg passing load
        float marker_alpha=0.5;
        // Set the marker type.
        uint32_t shape = visualization_msgs::Marker::CUBE;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 1;
        marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = -x_p(0);
        marker.pose.position.y = -x_p(1);
        marker.pose.position.z = -x_p(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = thresh2*300;
        marker.scale.y = thresh2*300;
        marker.scale.z = thresh2;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = marker_alpha;

        marker.lifetime = ros::Duration();
        marker.header.frame_id =frame_str;
        marker.header.stamp = ros::Time::now();

        if (j==0)
        {
            marker1=marker;
            marker1.pose.orientation.x = qx;
            marker1.pose.orientation.y = qy;
            marker1.pose.orientation.z = qz;
            marker1.pose.orientation.w = qw;
            marker1.color.r = 1.0f;
            marker1.color.g = 0.0f;
            marker1.color.b = 0.0f;
            marker1.color.a = marker_alpha;
        }
        else if(j==1)
        {
            marker2=marker;
            marker2.pose.orientation.x = qx;
            marker2.pose.orientation.y = qy;
            marker2.pose.orientation.z = qz;
            marker2.pose.orientation.w = qw;
            marker2.color.r = 0.0f;
            marker2.color.g = 1.0f;
            marker2.color.b = 0.0f;
            marker2.color.a = marker_alpha;
        }
        else
        {
            marker3=marker;
            marker3.pose.orientation.x = qx;
            marker3.pose.orientation.y = qy;
            marker3.pose.orientation.z = qz;
            marker3.pose.orientation.w = qw;
            marker3.color.r = 0.0f;
            marker3.color.g = 0.0f;
            marker3.color.b = 1.0f;
            marker3.color.a = marker_alpha;
        }

        std::cout << "Quaternion X: "<< qx << std::endl;
        std::cout << "Quaternion Y: "<< qy << std::endl;
        std::cout << "Quaternion Z: "<< qz << std::endl;
        std::cout << "Quaternion W: "<< qw << std::endl;

        j++;
    }

    myfile<<A1<<","<<B1<<","<<C1<<","<<D1<<","<<A2<<","<<B2<<","<<C2<<","<<D2<<","<<A3<<","<<B3<<","<<C3<<","<<D3<<std::endl;

    cloud_line->header.frame_id=frame_str;
    point_out.header.frame_id=frame_str;
    cloud_in->header.frame_id = frame_str;
    cloud_out1->header.frame_id = frame_str;
    cloud_out2->header.frame_id = frame_str;
    cloud_out3->header.frame_id = frame_str;

    //publish forever
    //cnt=0;
    while(ros::ok())
    {
        pub0.publish(cloud_in);
        pub1.publish(cloud_out1);
        pub2.publish(cloud_out2);
        pub3.publish(cloud_out3);

        pub4.publish(point_out);

        pub5.publish(cloud_line);

        pub6.publish(marker1);
        pub7.publish(marker2);
        pub8.publish(marker3);

        ros::spinOnce();
        loop_rate.sleep();
        //cnt++;
    }
    myfile.close();
    return 0;
}

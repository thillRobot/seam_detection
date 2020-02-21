/*
RANSAC/Segementation based multiple plane/line/point detection using PCL

Tristan Hill - Weld Seam Detection - Tennessee Technological University 

Taken from PCL sample code - 02/14/2018 
Updated - 02/17/2018 - 02/20/2018  - added solutions for intersection of planes
Updated - 02/26/2018  - adding 'visualization markers' for the planes - from the example code online

Updated - 03/07/2018 - adding lots of stuff, docs are out of date

Updated - 03/20/2018 - double filter, aka thick-thin filter is working with scene1

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
#include <math.h> 

#include <tf/transform_datatypes.h>
//#include <Quaternion.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"seam_detection_RANSAC"); 
    ros::NodeHandle node; 
    ros::Publisher pub0 = node.advertise<PointCloud> ("/cloud_in", 1) ;
    ros::Publisher pub1 = node.advertise<PointCloud> ("/cloud_out1", 1) ;
    ros::Publisher pub2 = node.advertise<PointCloud> ("/cloud_out2", 1) ;
    ros::Publisher pub3 = node.advertise<PointCloud> ("/cloud_out3", 1) ;

    ros::Publisher pub4 = node.advertise<geometry_msgs::PointStamped> ("/point_out", 1) ;

    ros::Publisher pub5 = node.advertise<PointCloud> ("/cloud_line", 1) ;

    //Pub for the plane marker
    ros::Publisher pub6 = node.advertise<visualization_msgs::Marker>("/plane_marker", 1);

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
    double thresh1 = atof(argv[3]);
    double thresh2 = atof(argv[4]);

    // This path must be changed when I switch workstations - TWH
    //std::string test_path = std::string("/home/bender/Dropbox/t410_ros/src/seam_detection/images/")+in_file;
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
    //seg.setDistanceThreshold(thresh);

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

    // add a 'visualization marker' for the planes - should decrease msg passing load
    visualization_msgs::Marker marker;
    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type. 
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DnverOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -x_p(0);
    marker.pose.position.y = -x_p(1);
    marker.pose.position.z = -x_p(2);

    double roll, pitch, yaw, alpha,beta,gamma;

   //yaw=acos(C3);                 //z
   // pitch=asin(B3/sin(acos(C3))); //x    
   // roll=0.0;                     //y  

    std::cout << "Setting up the null space problem." << std::endl;

    float Z1,Z2,Z3,X1,X2,X3,Y1,Y2,Y3;

    //Z1=A1;  // this shows the plane 2 (green)   (Why are planes 2 and 2 switched!!!)
    //Z2=B1;  // with - beta,alpha,gamma sol0
    //Z3=C1;

    Z1=A2; // this shows plane 1 (red)
    Z2=B2; // with - beta,alpha,gamma sol0
    Z3=C2;

    //Z1=A3;  // this shows plane 3 (blue)
    //Z2=B3;  // with - beta,alpha,gamma sol0
    //Z3=C3;
        

    Eigen::MatrixXf A6(6,6);
    /*
    A6<<Z1 ,Z2 ,Z3 ,0 ,0 ,0 ,       // comes from X dot Z equals zero
        0  ,0  ,0  ,Z1,Z2,Z3,       // and from Y = Z cross X
        -1  ,0  ,0  ,0 ,Z3,-Z2,
        0  ,-Z3,Z2,-1,0 ,0 ,
        -Z3,0  ,Z1 ,0 ,1 ,0 ,
        -Z2,Z1 ,0  ,0 ,0 ,-1 ;
    */
    A6<<0  ,-Z3,Z2,-1 ,0  , 0,      // comes from Y = Z cross X
        -Z3,0  ,Z1, 0 ,1  , 0,      // and        X = Y cross Z  
        -Z2,Z1 ,0 , 0 ,0  ,-1,
        -1 ,0  ,0 , 0 ,Z3 ,-Z2,
        0  ,1  ,0 , Z3,0  ,-Z1,
        0  ,0  ,-1, Z2,-Z1,0 ;

    std::cout << "The matrices have been defined." << std::endl;

    Eigen::FullPivLU<Eigen::MatrixXf> lu(A6);
    Eigen::MatrixXf A_ns = lu.kernel();

    std::cout << "The null space solution is:\n" << A_ns << std::endl;

    X1=A_ns(0,0);
    X2=A_ns(1,0);
    X3=A_ns(2,0);
    
    Y1=A_ns(3,0);
    Y2=A_ns(4,0);
    Y3=A_ns(5,0);    
    
    /*
    X1=0;

    Eigen::MatrixXf A5(5,5);
    Eigen::VectorXf b5(5),x5(5);

    A5<<Z2 ,Z3 ,0 ,0 ,0,
        0  ,0  ,Z1,Z2,Z3,
        -Z3,Z2,-1,0 ,0 ,
        0  ,Z1 ,0 ,1 ,0 ,
        Z1 ,0  ,0 ,0 ,-1 ;

    b5<<-X1*Z1,0,0,X1*Z3,X1*Z2;    
    
    std::cout << "The matrices have been defined." << std::endl;

    x5 = A5.colPivHouseholderQr().solve(b5); // There are several solvers to choose 
    std::cout << "The solution x is:\n" << x5 << std::endl;
    
    X2=x5(0,0);
    X3=x5(1,0);
    
    Y1=x5(2,0);
    Y2=x5(3,0);
    Y3=x5(4,0);
    */

    //Eigen::FullPivLU<Eigen::MatrixXf> lu(A6);
    //Eigen::MatrixXf A_null_space = lu.kernel();

    //std::cout << "The null space solution is:\n" << A_null_space << std::endl;
    /*
    X1=A_null_space(0,0);
    X2=A_null_space(1,0);
    X3=A_null_space(2,0);
    
    Y1=A_null_space(3,0);
    Y2=A_null_space(4,0);
    Y3=A_null_space(5,0); 
    */    
    //X3=(-X1*Z1-X2*Z2)/Z3;
    //Y1=Z2*X3-Z3*X2;
        
    // these come from (https://www.mathworks.com/matlabcentral/answers/298940-how-to-calculate-roll-pitch-and-yaw-from-xyz-coordinates-of-3-planar-points) -     
   // alpha= atan2(-Z2, Z3); //x
   // beta= asin(Z1);        //y
   // gamma = atan2(-Y1,X1); //z
    
    // these come from (http://planning.cs.uiuc.edu/node103.html) - looks legit :)
    alpha= atan2(X2,X1); //x 
    beta = atan2(-X3,sqrt( pow(Y3,2)+pow(Z3,2)) ); //y
    gamma= atan2(Y3,Z3); //z   
        
    std::cout << "Alpha: "<< alpha << std::endl;
    std::cout << "Beta: "<< beta << std::endl;
    std::cout << "Gamma: "<< gamma << std::endl;
    //this is close at least but something is switched
    roll=beta;            //z
    pitch=alpha;          //y
    yaw=gamma;            //x
    
    // convert from RPY to Quaternions - (from wikipedia)
    double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

    marker.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    marker.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    marker.pose.orientation.z = sy * cr * cp - cy * sr * sp;
    marker.pose.orientation.w = cy * cr * cp + sy * sr * sp;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = thresh2;
    marker.scale.y = thresh2*300;
    marker.scale.z = thresh2*300;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
  
    // set the fixed reference frame - for now they are all the same
    std::string frame_str="map";
    
    cloud_line->header.frame_id=frame_str;
    point_out.header.frame_id=frame_str;
    cloud_in->header.frame_id = frame_str;
    cloud_out1->header.frame_id = frame_str;
    cloud_out2->header.frame_id = frame_str;
    cloud_out3->header.frame_id = frame_str;
    
    // not pointers !!!
    marker.header.frame_id =frame_str;
    marker.header.stamp = ros::Time::now();

    //publish forever
    while(ros::ok())
    {    
        pub0.publish(cloud_in);
        pub1.publish(cloud_out1);
        pub2.publish(cloud_out2);
        pub3.publish(cloud_out3);        
        
        pub4.publish(point_out);

        pub5.publish(cloud_line); 

        pub6.publish(marker);    

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


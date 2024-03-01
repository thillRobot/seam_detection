#ifndef CLOUDFILTER_H
#define CLOUDFILTER_H

#include <string>
#include <ros/ros.h>


class CloudFilter
{

  private:
    
    std::string config;

    ros::NodeHandle node;

  public: 
     
    CloudFilter(); 
    CloudFilter(std::string);
    ~CloudFilter();  

    void loadConfig(std::string cfg);

    std::string getConfig(void);
   
};


#endif

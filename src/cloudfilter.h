#ifndef CLOUDFILTER_H
#define CLOUDFILTER_H

#include <string> 
#include <ros/ros.h>

class CloudFilter
{

  private:

    std::string config;

    ros::NodeHandle node;
    
    void setConfig(const std::string cfg);
    
    int loadConfig(void);

  public: 
      
    CloudFilter();
    
    CloudFilter(const std::string cfg = "cloudfilter");
    
    std::string getConfig(void);
    
};


#endif

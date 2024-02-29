#ifndef CLOUDFILTER_H
#define CLOUDFILTER_H

#include <string>

class CloudFilter
{

  private:
    
    int foo;    
    std::string config;

  public: 
      
    CloudFilter();
    
    void loadConfig(std::string cfg);

    std::string getConfig(void);
   
};


#endif

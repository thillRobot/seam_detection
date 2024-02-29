#include "cloudfilter.h"
#include <iostream>

int main(){

  CloudFilter filter;   

  filter.loadConfig("filtercloud_test");

  std::cout<<"loading config: "<<filter.getConfig()<<std::endl;
 
  return 0;

}

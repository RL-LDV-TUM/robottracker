#include <iostream>

#include "RobotTraffic.hpp"
#include "TrafficServer.hpp"

int main(int argc,char **argv)
{
    try
    {
        if (argc <= 0)
        {
          cerr<<"Invalid number of arguments:"<<endl;
          cerr<<"\tUsage: tba."<<endl;
          return false;
        }
        
        RobotTraffic robotTraffic;
        TrafficServer tserver(robotTraffic);
        tserver.run();
        
        tserver.shutDown();
        
       
    } catch (std::exception &ex)
    {
        std::cout<<"Exception :"<<ex.what()<<std::endl;
    }
}

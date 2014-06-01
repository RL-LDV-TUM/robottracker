#include <iostream>

#include "TrafficServer.hpp"

int main(int argc,char **argv)
{
    try
    {
       
        TrafficServer tserver;
        tserver.run();


    } catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}

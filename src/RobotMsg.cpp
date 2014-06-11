#include <iostream>
#include <arpa/inet.h>

#include "RobotMsg.hpp"

/*
* Some helper functions to handle RobotMessages
*/
namespace RobotMessage
{

  void print(const RobotMsg *msg)
  {
    std::cout << "Id:" << msg->id << std::endl 
    << "Sec:" << msg->sec  << std::endl
    << "X:" << msg->x <<" Y:" <<  msg->y <<" Z:" <<  msg->z << std::endl
    << "xCell:" << msg->xCell << " yCell:" << msg->yCell << std::endl
    << "Angle:" << msg->angle << std::endl;
  }

  void hton(RobotMsg *msg) {
    msg->id = htonl(msg->id);
    msg->sec = htonl(msg->sec);
    msg->x = htonl(msg->x);
    msg->y = htonl(msg->y);
    msg->z = htonl(msg->z);
    msg->xCell = htonl(msg->xCell);
    msg->yCell = htonl(msg->yCell);
    msg->angle = htonl(msg->angle);
    
  }

  void ntoh(RobotMsg *msg) {
    msg->id = ntohl(msg->id);
    msg->sec = ntohl(msg->sec);
    msg->x = ntohl(msg->x);
    msg->y = ntohl(msg->y);
    msg->z = ntohl(msg->z);
    msg->xCell = ntohl(msg->xCell);
    msg->yCell = ntohl(msg->yCell);
    msg->angle = ntohl(msg->angle);
  }

}

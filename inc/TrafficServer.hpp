#ifndef _TrafficServer_H_
#define _TrafficServer_H_

#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "RobotTraffic.hpp"

class TrafficServer {


  public:
  
    TrafficServer(unsigned port = 5005);
    ~TrafficServer();
    
    /*
    * Set reference to RobotTraffic
    * used for robot lookups
    */
    void setRobotTraffic(const RobotTraffic * traffic) { robotTraffic = traffic; }
    
    /*
    * Run server loop
    */
    void run();
    
    /*
    * Shutdown server
    */
    void shutDown();
    

  protected:
    
    unsigned sockfd, newsockfd, portno, clilen;
    struct sockaddr_in serv_addr, cli_addr;
    
    void init();
    
    /*
    * Exchange messages
    */
    void communicate(unsigned sock);
    
    // ptr to constant RobotTraffic (disallow write from here)
    const RobotTraffic * robotTraffic = 0;
    
};

#endif

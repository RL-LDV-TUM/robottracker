#ifndef _TrafficServer_H_
#define _TrafficServer_H_

#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "RobotTraffic.hpp"

class TrafficServer {


  public:
  
    TrafficServer(RobotTraffic &robotTraffic, unsigned port = 5005);
    ~TrafficServer();
    
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
    
    // RobotTraffic for lookups
    RobotTraffic &robotTraffic;
    
};

#endif

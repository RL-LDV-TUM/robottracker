#include "TrafficServer.hpp"

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <strings.h>
#include <unistd.h>
#include <thread>

TrafficServer::TrafficServer(RobotTraffic &robotTraffic, unsigned port) : robotTraffic(robotTraffic), portno(port)
{
    init();
}

TrafficServer::~TrafficServer()
{

}

void TrafficServer::init()
{

    // First call to socket() function
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    {
        std::cerr << "TrafficServer: ERROR opening socket" << std::endl;
        exit(1);
    }
    // Initialize socket structure
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
 
    // Now bind the host address using bind() call.
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
         std::cerr << "TrafficServer: ERROR on binding" << std::endl;
         exit(1);
    }
    /* Now start listening for the clients, here 
     * thread will go in sleep mode and will wait 
     * for the incoming connection 
     */
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    
}

/*
* Run server loop
*/
void TrafficServer::run()
{
  std::cout << "TrafficServer: Started" << std::endl;

  while (sockfd) 
  {
      newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
      
      if (newsockfd != -1)
      {
        // Create comm thread
        std::thread commThread (&TrafficServer::communicate, *this, newsockfd );
        commThread.detach();
      } else {
        std::cout << "TrafficServer: Stopped" << std::endl;
        break;
      }
  }
  
}

/*
* Shutdown server
*/
void TrafficServer::shutDown()
{
   shutdown(sockfd, 2);
   close(sockfd);
}

/*
* Exchange messages
*/
void TrafficServer::communicate(unsigned sock)
{
    int n;

    int robotId;
    
    while(true)
    {

      n = read(sock,&robotId,sizeof(robotId));
      if (n < 0)
      {
          break;
      }
      
      RobotMsg msg = robotTraffic.queryRobot(robotId);
      
      // network byte order
      // RobotMessage::hton(&msg);
      
      n = write(sock, &msg, sizeof(msg));
      std::cout << "Msg (" << sizeof(msg) << " bytes) sent for Robot " << robotId << std::endl;
      
      if (n < 0) 
      {
          break;
      }
    }
    
    close(sock);
}

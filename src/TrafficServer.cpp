#include "TrafficServer.hpp"

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <strings.h>
#include <unistd.h>
#include <thread>

TrafficServer::TrafficServer(unsigned port) : portno(port)
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
     * process will go in sleep mode and will wait 
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
  while (sockfd) 
  {
      newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
      
      if (newsockfd != -1)
      {
        // Create comm thread
        std::thread commThread (&TrafficServer::communicate, *this, newsockfd );
        commThread.detach();
      } else {
        std::cout << "TrafficServer: closed" << std::endl;
        exit(0);
      }
  }
  
}

/*
* Shutdown server
*/
void TrafficServer::shutDown()
{
   shutdown(sockfd, 2);
}

/*
* Exchange messages
*/
void TrafficServer::communicate(unsigned sock)
{
    int n;
    char buffer[256];

    bzero(buffer,256);

    n = read(sock,buffer,255);
    if (n < 0)
    {
        std::cerr << "TrafficServer: ERROR reading from socket" << std::endl;
        exit(1);
    }
    
    RobotTrace trace;
    int robotId = atoi(buffer);
    
    if(robotTraffic)
    {
      trace = robotTraffic->queryRobot(robotId);
    } else {
      trace = RobotTrace(cv::Mat::eye(4, 4, CV_32F), 0);
    }
    
    std::stringstream msg;
    msg << robotId << ":" << trace.first << trace.second << endl;
    std::string msg_str = msg.str();
    
    n = write(sock, msg_str.c_str(), msg_str.length());
    
    if (n < 0) 
    {
        std::cerr << "TrafficServer: ERROR writing to socket" << std::endl;
        exit(1);
    }
    
    close(sock);
}

#include <stdio.h>
#include <iostream>
#include <strings.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "RobotMsg.hpp"

int main(int argc,char **argv)
{
    try
    {
        int sockfd, portno, n;
        struct sockaddr_in serv_addr;
        struct hostent *server;
        
        portno = 5005;
        std::string hostname = "127.0.0.1";
        
        
        /* Create a socket point */
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) 
        {
            perror("ERROR opening socket");
            exit(1);
        }
        
        server = gethostbyname(hostname.c_str());
        
        if (server == NULL) {
            fprintf(stderr,"ERROR, no such host\n");
            exit(0);
        }

        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,
                    server->h_length);
        serv_addr.sin_port = htons(portno);

        /* Now connect to the server */
        if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        {
             perror("ERROR connecting");
             exit(1);
        }	
        
        while(true)
        {
        
          /* Now ask for a message from the user, this message
          * will be read by server
          */
          std::cout << "Enter Robot ID:";
          
          int robotId = 0;
          std::cin >> robotId;
          
          if(robotId <= 0) {
            std::cin.clear();
		        std::cin.ignore(256, '\n');
            continue;
          }
          
          /* Send message to the server */
          n = write(sockfd,(void*) &robotId,sizeof(robotId));
          if (n <= 0) 
          {
               perror("ERROR writing to socket");
               exit(1);
          }
          
          /* Now read server response */
          RobotMsg rmsg;
          n = read(sockfd,&rmsg,sizeof(rmsg));
          if (n <= 0) 
          {
               perror("ERROR reading from socket");
               exit(1);
          }
          
          // show us the result
          RobotMessage::print(&rmsg);
        }
        
        return 0;
        
        
    } catch (std::exception &ex)
    {
        std::cout<<"Exception :"<<ex.what()<<std::endl;
    }
}

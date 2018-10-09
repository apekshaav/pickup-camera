#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <ros/ros.h>

typedef struct sensor_pose
{
    float xpos;
    float ypos;
    float zpos;

    float qw;
    float qx;
    float qy;
    float qz;
}sensor_pose;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
    // initialize socket variables
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    // char buffer[6];
    
    // create socket
    portno = 12345;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");

    server = gethostbyname("localhost");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    // set server details
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    
    // connect to server
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    ROS_INFO("Connected");
    
    // bzero(buffer,256);
    // creating new pose
    sensor_pose newpose;
    newpose.xpos = 0.01;
    newpose.ypos = 0.0;
    newpose.zpos = 0.0;

    newpose.qw = 1.0;
    newpose.qx = 0.17;
    newpose.qy = 0.17;
    newpose.qz = 0.0;
    
    // write message to socket
    n = write(sockfd, (const char*)&newpose, 28);
    if (n < 0) 
         error("ERROR writing to socket");

    close(sockfd);

    return 0;
}
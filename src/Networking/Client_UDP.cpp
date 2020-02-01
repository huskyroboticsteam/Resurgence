// UDP client program 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <stdio.h> 
#include <stdlib.h>
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <iostream>
#define PORT 5000 
#define MAXLINE 1024 
int main() 
{ 
	int sockfd; 
	struct sockaddr_in servaddr; 

	// Creating socket file descriptor 
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
		printf("socket creation failed"); 
		exit(0); 
	} 

	memset(&servaddr, 0, sizeof(servaddr)); 

	// Filling server information 
	servaddr.sin_family = AF_INET; 
	servaddr.sin_port = htons(PORT); 
	servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 

	std::string str;
	char buffer[MAXLINE]; 

	int n;
	socklen_t len; 

	while (1) {
		std::cout << "Enter message: ";
		std::cin >> str;

		if (str == "exit")
			break;
		
		// send hello message to server 
		sendto(sockfd, str.c_str(), str.length(), 
			0, (const struct sockaddr*)&servaddr, 
			sizeof(servaddr)); 

		bzero(buffer, sizeof(buffer));

		// receive server's response 
		printf("Message from server: "); 
		n = recvfrom(sockfd, buffer, MAXLINE, 
					0, (struct sockaddr*)&servaddr, 
					&len); 
		puts(buffer); 
	}

	close(sockfd); 
	return 0; 
} 

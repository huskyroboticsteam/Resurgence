// TCP Client program 
#include <netinet/in.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <iostream>
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <arpa/inet.h>
#define PORT 5000 
#define MAXLINE 1024 
int main() 
{ 
	int sockfd; 
	struct sockaddr_in servaddr; 

	int n, len; 
	// Creating socket file descriptor 
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) { 
		printf("socket creation failed"); 
		exit(0); 
	} 

	memset(&servaddr, 0, sizeof(servaddr)); 

	// Filling server information 
	servaddr.sin_family = AF_INET; 
	servaddr.sin_port = htons(PORT); 
	servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 

	if (connect(sockfd, (struct sockaddr*)&servaddr, 
							sizeof(servaddr)) < 0) { 
		printf("\n Error : Connect Failed \n"); 
	} 

	std::string str;
	char buffer[MAXLINE]; 

	while (1) {
		std::cout << "Enter message: ";
		std::cin >> str;

		if (str == "exit")
			break;

		write(sockfd, str.c_str(), sizeof(buffer)); 
		printf("Message from server: "); 
		bzero(buffer, sizeof(buffer));
		read(sockfd, buffer, sizeof(buffer)); 
		puts(buffer); 
	}

	close(sockfd); 
} 

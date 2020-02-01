// TCP Client program
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

#define PORT 5000 
#define MAXLINE 1024

int main()
{
	int sockfd;
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		std::cout << "socket creation failed";
		exit(0); 
	}

	struct sockaddr_in servaddr;
	bzero(&servaddr, sizeof(servaddr));

	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
	servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // set this to whatever the server's IP is

	if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
		std::cout << "Error : Connect Failed" << std::endl;
	}

	std::string str;
	char buffer[MAXLINE]; 

	while (1) {
		std::cout << "Enter message: ";
		std::getline(std::cin, str);

		if (str == "exit") {
			break;
		}

		write(sockfd, str.c_str(), str.length());

		bzero(buffer, sizeof(buffer));
		read(sockfd, buffer, sizeof(buffer));

		std::cout << "Message from server: " << buffer << std::endl;
	}

	close(sockfd);
}

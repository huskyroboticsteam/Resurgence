#include "NetworkConstants.h"

int main()
{
	int sockfd;

	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		std::cout << "socket creation failed";
		exit(0);
	}

	struct sockaddr_in servaddr;
	bzero(&servaddr, sizeof(servaddr));

	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
	servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);

	std::string str;
	char buffer[MAXLINE];

	int n;
	socklen_t len;

	while (1) {
		std::cout << "Enter message: ";
		std::getline(std::cin, str);

		if (str == "exit") {
			break;
		}
		
		sendto(sockfd, str.c_str(), str.length(), 0, (const struct sockaddr*)&servaddr, sizeof(servaddr));

		bzero(buffer, sizeof(buffer));
		n = recvfrom(sockfd, buffer, MAXLINE, 0, (struct sockaddr*)&servaddr, &len);
		std::cout << "Message from server: " << buffer << std::endl;
	}

	close(sockfd);
} 

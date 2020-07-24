#include "NetworkConstants.h"

int max(int x, int y) {
	if (x > y)
		return x;
	else
		return y;
}

int main()
{
	int listenfd, connfd, udpfd, nready, maxfdp1;
	char buffer[MAXLINE];
	pid_t childpid;
	fd_set rset;
	ssize_t n;
	socklen_t len;
	const int on = 1;
	struct sockaddr_in cliaddr, servaddr;
	void sig_chld(int);

	/* create listening TCP socket */
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(PORT);

	// binding server addr structure to listenfd
	bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr));
	listen(listenfd, 10);

	/* create UDP socket */
	udpfd = socket(AF_INET, SOCK_DGRAM, 0);
	// binding server addr structure to udp sockfd
	bind(udpfd, (struct sockaddr*)&servaddr, sizeof(servaddr));

	// clear the descriptor set
	FD_ZERO(&rset);

	// get maxfd
	maxfdp1 = max(listenfd, udpfd) + 1;
	while (1) {
		// set listenfd and udpfd in readset
		FD_SET(listenfd, &rset);
		FD_SET(udpfd, &rset);

		// select the ready descriptor
		nready = select(maxfdp1, &rset, NULL, NULL, NULL);

		// if tcp socket is readable then handle
		// it by accepting the connection
		if (FD_ISSET(listenfd, &rset)) {
			len = sizeof(cliaddr); 
			connfd = accept(listenfd, (struct sockaddr*)&cliaddr, &len);
			if ((childpid = fork()) == 0) {
				close(listenfd);

				while (1) {
					bzero(buffer, sizeof(buffer));
					n = read(connfd, buffer, sizeof(buffer));
					if (n < 0 || strcmp(buffer, CLOSE_TCP) == 0) {
						break;
					}
					std::cout << "Message From TCP client: " << buffer << std::endl;

					std::string str(buffer);
					str = "Echoing \"" + str + "\"";
					write(connfd, str.c_str(), str.length());
				}

				close(connfd);
				exit(0);
			}
			
			close(connfd);
		}

		// if udp socket is readable receive the message. 
		if (FD_ISSET(udpfd, &rset)) {
			len = sizeof(cliaddr);
			bzero(buffer, sizeof(buffer));
			n = recvfrom(udpfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);
			std::cout << "Message from UDP client: " << buffer << std::endl;

			std::string str(buffer);
			str = "Echoing \"" + str + "\"";
			sendto(udpfd, str.c_str(), str.length(), 0, (struct sockaddr*)&cliaddr, sizeof(cliaddr));
		}
	}
}

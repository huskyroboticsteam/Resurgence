#include "NetworkConstants.h"

int main()
{
	int listenfd, connfd;
	char buffer[MAXLINE];
	pid_t childpid;
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

  std::cout << "Waiting for rover....\n";
	while (1) {
    len = sizeof(cliaddr);
    connfd = accept(listenfd, (struct sockaddr*)&cliaddr, &len);
    if ((childpid = fork()) == 0) {
      std::cout << "Connected to rover.\n";
      close(listenfd);

      std::string str;
      while (1) {
        std::cout << "Enter message in JSON format (no newlines): ";
        std::getline(std::cin, str);
        write(connfd, str.c_str(), str.length());

        bzero(buffer, sizeof(buffer));
        n = read(connfd, buffer, sizeof(buffer));
        if (n <= 0 || strcmp(buffer, CLOSE_TCP) == 0) {
          std::cout << "Disconnected from rover.\n";
          break;
        }
        std::cout << "Message from rover: " << buffer << std::endl;
      }

      close(connfd);
      exit(0);
    }
    close(connfd);
	}
}

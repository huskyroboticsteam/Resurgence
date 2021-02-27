#include "NetworkConstants.h"
#include <csignal>

int listenfd;

void cleanup(int signum) {
  std::cout << "Interrupted\n";
  close(listenfd);
  exit(0);
}

int main()
{
  int connfd;
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

  signal(SIGINT, cleanup);

  // binding server addr structure to listenfd
  if (bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
    perror("Could not bind");
    exit(1);
  }
  if (listen(listenfd, 10) < 0) {
    perror("Could not listen");
    exit(1);
  }

  std::cout << "Waiting for rover....\n";
  while (1)
  {
    len = sizeof(cliaddr);
    if ((connfd = accept(listenfd, (struct sockaddr*)&cliaddr, &len)) < 0) {
      perror("Could not open listening socket");
      exit(1);
    }
    if ((childpid = fork()) == 0)
    {
      std::cout << "Connected to rover.\n";
      close(listenfd);

      std::string str;
      while (1)
      {
        std::cout << "Enter message in JSON format (no newlines): ";
        std::getline(std::cin, str);
        int msg_len = str.length();
        int ret = write(connfd, &msg_len, 4);
        if (ret < 0) {
          perror("Socket write failed");
        }
        else if (ret != 4) {
          perror("Wrote a weird number of bytes");
        }
        if (write(connfd, str.c_str(), str.length()) < 0) {
          perror("Socket write failed");
        }

        bzero(buffer, sizeof(buffer));
        if ((n = read(connfd, buffer, sizeof(buffer))) < 0) {
          perror("Socket read failed");
        }
        if (n <= 0 || strcmp(buffer, CLOSE_TCP) == 0)
        {
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

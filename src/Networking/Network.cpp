#include "../Globals.h"
#include "Network.h"
#include "ParseCAN.h"
#include "NetworkConstants.h"

#include <cassert>

#include <net/if.h>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <aio.h>

#include<iostream>

int base_station_fd;

void InitializeBaseStationSocket()
{
  if ((base_station_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("Base station socket creation failed");
    exit(1);
  }

  struct sockaddr_in servaddr;
  bzero(&servaddr, sizeof(servaddr));

  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(PORT);
  servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);

  if (connect(base_station_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
  {
    perror("Base station connect failed");
    exit(1);
  }
}

void recvBaseStationPacket()
{
  char buffer[MAXLINE];
  bzero(buffer, sizeof(buffer));
  std::cout << "Waiting for base station packet" << std::endl;
  if (read(base_station_fd, buffer, sizeof(buffer)) < 0)
  {
    perror("Failed to read from base station");
  }
  else
  {
    std::cout << "Message from base station: " << buffer << std::endl;
    nlohmann::json parsed_message = nlohmann::json::parse(buffer);
    // TODO proper input validation. Here we assume we got a dict with key "speed"
    std::cout << parsed_message["speed"] << std::endl;
  }
  std::string serialized_status = Globals::status_data.dump();
  if (write(base_station_fd, serialized_status.c_str(), serialized_status.length()) < 0)
  {
    perror("Failed to send to base station");
  }
}

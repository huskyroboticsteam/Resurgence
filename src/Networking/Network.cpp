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

#include<iostream>

int base_station_fd = -1;
bool connected = false;

bool InitializeBaseStationSocket()
{
  if (connected) return connected;

  if ((base_station_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("Base station socket creation failed");
    return false;
  }

  struct sockaddr_in servaddr;
  bzero(&servaddr, sizeof(servaddr));

  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(PORT);
  servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);

  if (connect(base_station_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
  {
    perror("Base station connect failed");
    return false;
  }

  return (connected = true);
}

int recvBaseStationPacket(char *buffer)
{
  if (!connected) return 0;

  // TODO: How do we split base station packets? If we split on newlines
  // this will require more complicated buffer management.
  // If we split by requiring the base station to send a four-byte length
  // before sending each packet, we might need to dynamically allocate
  // memory. Either way we may need to handle blocking reads more carefully.
  uint8_t len_buffer[4];
  int ret = recv(base_station_fd, &len_buffer, 4, MSG_DONTWAIT);
  if (ret < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      perror("Failed to read from base station");
    }
    return 0;
  } else if (ret == 0) {
    perror("Base station disconnected");
    exit(1);
  } else if (ret != 4) {
    printf("Could not read exactly four bytes from base station (got %d)", ret);
    exit(1);
  }
  uint8_t b0 = len_buffer[0];
  uint8_t b1 = len_buffer[1];
  uint8_t b2 = len_buffer[2];
  uint8_t b3 = len_buffer[3];
  //printf("received bytes from base station: %x %x %x %x\n", b0, b1, b2, b3);
  int length = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
  //printf("parsed this to packet length: %d\n", length);

  if (length > MAXLINE-1)
  {
    printf("Message length from base station is too large: %d", length);
    exit(1);
  }

  ret = recv(base_station_fd, buffer, length, MSG_DONTWAIT);

  if (ret < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      perror("Failed to read from base station");
    }
    return 0;
  } else if (ret == 0) {
    perror("Base station disconnected");
    exit(1);
  } else if (ret != length) {
    printf("Could not read %d bytes from the base station (got %d)", length, ret);
    exit(1);
  }
  //printf("final two characters of message: %d %d '%s'", buffer[length-2], buffer[length-1], buffer + (length-2));

  return 1;
}

void sendBaseStationPacket(const std::string &packet) {
  if (write(base_station_fd, packet.c_str(), packet.length()) < 0)
  {
    perror("Failed to send to base station");
  }
}

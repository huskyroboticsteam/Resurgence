#include "NetworkConstants.h"
extern "C"
{
    #include "../HindsightCAN/CANPacket.h"
}

#include <net/if.h>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

struct sockaddr_can can_addr;
struct can_frame can_frame_;
struct ifreq can_ifr;

#include<iostream>

int can_fd;

void error(const char *msg) {
  perror(msg);
  exit(1);
}

void InitializeCANSocket()
{
  if((can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    error("Failed to initialize CAN bus!");
  }

  strcpy(can_ifr.ifr_name, "can0");
  if (ioctl(can_fd, SIOCGIFINDEX, &can_ifr) < 0)
  {
    perror("Failed to get hardware CAN interface index");
    strcpy(can_ifr.ifr_name, "vcan0");
    if (ioctl(can_fd, SIOCGIFINDEX, &can_ifr) < 0)
    {
      // You can make a virtual CAN interface with
      //
      // sudo ip link add type vcan
      // sudo ip link set dev vcan0 up
      //
      error("Failed to get virtual CAN interface index");
    }
    std::cout << "Found virtual CAN interface index." << std::endl;
  }

  can_addr.can_family = AF_CAN;
  std::cout << "Index: " << can_ifr.ifr_ifindex << std::endl;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;

  if (bind(can_fd, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0)
  {
    error("Failed to bind CAN socket");
  }
}

void SendCANPacket(const CANPacket &packet)
{
  // TODO why do we have our own custom struct for CAN packets? Should just use `struct can_frame`.
  can_frame_.can_id = packet.id;
  can_frame_.can_dlc = packet.dlc;
  for(int i = 0; i < packet.dlc; i++)
  {
      can_frame_.data[i] = packet.data[i];
  }

  if (sendto(can_fd, &can_frame_, sizeof(struct can_frame),
        0, (struct sockaddr*)&can_addr, sizeof(can_addr)) < 0)
  {
    perror("Failed to send CAN packet");
  }
  else
  {
    std::cout << "CAN packet sent.\n";
  }
}

int recvCANPacket(CANPacket *packet)
{
  socklen_t len = sizeof(can_addr);

  if (recvfrom(can_fd, &can_frame_, sizeof(struct can_frame),
                MSG_DONTWAIT, (struct sockaddr*)&can_addr, &len) < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      perror("Failed to receive CAN packet");
    }
    return 0;
  }
  else
  {
    std::cout << "Got CAN packet" << std::endl;
    packet->id = can_frame_.can_id;
    packet->dlc = can_frame_.can_dlc;
    for(int i = 0; i < can_frame_.can_dlc; i++)
    {
        packet->data[i] = can_frame_.data[i];
    }
    return 1;
  }
}

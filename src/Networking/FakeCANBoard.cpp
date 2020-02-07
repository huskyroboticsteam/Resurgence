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
#include <aio.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <aio.h>

struct sockaddr_can can_addr;
struct can_frame can_frame_;
struct ifreq can_ifr;

#include<iostream>

int can_fd;

void error(const char *msg) {
  std::cout << msg << std::endl;
  exit(1);
}

void InitializeCANSocket()
{
  if((can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    error("Failed to initialize CAN bus!");
  }

  strcpy(can_ifr.ifr_name, "can0");
  ioctl(can_fd, SIOCGIFINDEX, &can_ifr);

  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;

  bind(can_fd, (struct sockaddr *)&can_addr, sizeof(can_addr));
}

void SendCANPacket()
{
  can_frame_.can_id = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_JETSON, DEVICE_SERIAL_JETSON);
  can_frame_.can_dlc = 7;
  WriteSenderSerialAndPacketID(can_frame_.data, 0x36); // 0x36 is telemetry report
  can_frame_.data[2] = 0x01; // current
  for(int i = 3; i < 8; i++){
      can_frame_.data[i] = 0;
  }
  can_frame_.data[5] = 0x01; // 256 milliamps

  sendto(can_fd, &can_frame_, sizeof(struct can_frame),
      0, (struct sockaddr*)&can_addr, sizeof(can_addr));
}

void recvCANPacket()
{
  socklen_t len = sizeof(can_addr);

  std::cout << "Watiing for CAN packet" << std::endl;
  recvfrom(can_fd, &can_frame_, sizeof(struct can_frame),
                0, (struct sockaddr*)&can_addr, &len);

  CANPacket packet;
  packet.id = can_frame_.can_id;

  for(int i = 0; i < can_frame_.can_dlc; i++){
      packet.data[i] = can_frame_.data[i];
  }
}

int main() {
  InitializeCANSocket();
  std::string str;
  std::cout << "Press enter to send a CAN packet > ";
  while(1) {
    std::getline(std::cin, str);
    SendCANPacket();
  }
}

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

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <aio.h>

struct sockaddr_can can_addr;
struct can_frame can_frame_;
struct ifreq can_ifr;
#endif

#include<iostream>

int can_fd;
int base_station_fd;

void error(const char *msg) {
  std::cout << msg << std::endl;
  exit(1);
}

void InitializeCANSocket()
{
  #ifdef __linux__

  if(can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW))
      error("Failed to initialize CAN bus!");

  strcpy(can_ifr.ifr_name, "can0");
  ioctl(can_fd, SIOCGIFINDEX, &can_ifr);

  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;

  bind(can_fd, (struct sockaddr *)&can_addr, sizeof(can_addr));

  #endif
}

void InitializeBaseStationSocket()
{
	if ((base_station_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    error("Base station socket creation failed");
	}

	struct sockaddr_in servaddr;
	bzero(&servaddr, sizeof(servaddr));

	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
	servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);

	if (connect(base_station_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
		error("Base station connect failed");
	}
}

void recvBaseStationPacket()
{
  char buffer[MAXLINE];
  bzero(buffer, sizeof(buffer));
  read(base_station_fd, buffer, sizeof(buffer));
  std::cout << buffer << std::endl;
}

// TODO(sasha): We probably want all of this to use asynchronous IO
//              Check out POSIX aio.
void recvCANPacket()
{
  #ifdef __linux__
  socklen_t len = sizeof(can_addr);

  // TODO: Probably should thread reading CAN
  recvfrom(can_fd, &can_frame_, sizeof(struct can_frame),
                0, (struct sockaddr*)&can_addr, &len);

  CANPacket packet;
  packet.id = can_frame_.can_id;

  for(int i = 0; i < can_frame_.can_dlc; i++){
      packet.data[i] = can_frame_.data[i];
  }

  ParseCANPacket(packet);

  #endif
}

void SendOutgoingNetworkPackets()
{
    for(const Packet &p : Globals::outgoing_packets)
    {
        if(p.kind == PacketKind::CAN){
            #ifdef __linux__

            can_frame_.can_id = p.address & 0x1FF;
            can_frame_.can_dlc = 8;
            for(int i = 0; i < 8; i++){
                can_frame_.data[i] = p.payload[i];
            }

            sendto(can_fd, &can_frame_, sizeof(struct can_frame),
                0, (struct sockaddr*)&can_addr, sizeof(can_addr));

            #endif
            //TODO Maybe?: Use aio
            //aio_write(can_fd, p);
        }
        else if(p.kind == PacketKind::Network)
            ;
            //aio_write(net_fd, p);
    }
}

// For testing uses
// Used to manually send can bytes one at a time
// Data is placed in outgoing packets list and will be processed in
// SendOutgoingNetworkPackets() function 
void TestCANPackets()
{
    Packet packet;
    packet.kind = PacketKind::CAN;
    std::cout << "Enter in Can ID" << std::endl;
    int id;
    std::cin >> id;
    packet.address = id;

    uint8_t data[Constants::PACKET_PAYLOAD_SIZE] = {0};
    data[1] = 1;
    data[2] = 2;
    for(int i = 0; i < 8; i++){
        packet.payload[i] = data[i];
    }


    Globals::outgoing_packets.push_back(packet);
    std::cout << "Input Cycle Complete" << std::endl;

}

// Send JSON data to Mission Control
void SendMissionControlStatus()
{
    // We're going to construct our json data here. We're going to use camel
    // case for the json data fields to conform to Mission Control style guide
    //nlohmann::json j;
    //j["timestamp"] = p.timestamp;
    //j["backLeftMotorCurrent"] = p.back_left_motor_current;
    // TODO: Finish constructing JSON here
}

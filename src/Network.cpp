#include "Globals.h"
#include "Network.h"
#include "CANPacket.h"

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

struct sockaddr_can addr;
struct can_frame frame;
struct ifreq ifr;
#endif

#include<iostream>

void InitializeNetwork()
{
    #ifdef __linux__

    if(Globals::can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW))
        assert(!"Failed to initialize CAN bus!");

    strcpy(ifr.ifr_name, "can0");
    ioctl(Globals::can_fd, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(Globals::can_fd, (struct sockaddr *)&addr, sizeof(addr));

    #endif
    
}

// TODO(sasha): We probably want all of this to use asynchronous IO
//              Check out POSIX aio.
void ParseIncomingNetworkPackets()
{

    // TODO: Get network packets
    
    #ifdef __linux__
    socklen_t len = sizeof(addr);

    // TODO: Probably should thread reading CAN
    recvfrom(Globals::can_fd, &frame, sizeof(struct can_frame),
                  0, (struct sockaddr*)&addr, &len);

    Packet packet;
    packet.kind = PacketKind::CAN;

    packet.address = frame.can_id;

    for(int i = 0; i < 8; i++){
        packet.payload[i] = frame.data[i];
    }

    Globals::incoming_packets.push_back(packet);    

    #endif

}

void SendOutgoingNetworkPackets()
{
    
    for(const Packet &p : Globals::outgoing_packets)
    {
        if(p.kind == PacketKind::CAN){
            #ifdef __linux__
            
            frame.can_id = p.address & 0x1FF;
            frame.can_dlc = 8;
            for(int i = 0; i < 8; i++){
                frame.data[i] = p.payload[i];
            }

            sendto(Globals::can_fd, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

            #endif
            //TODO Maybe?: Use aio
            //aio_write(Globals::can_fd, p);
        }
        else if(p.kind == PacketKind::Network)
            ;
            //aio_write(Globals::net_fd, p);
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

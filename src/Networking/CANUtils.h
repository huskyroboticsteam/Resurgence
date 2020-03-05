
#ifndef __CAN_UTILS_H__
#define __CAN_UTILS_H__

extern "C"
{
    #include "../HindsightCAN/CANPacket.h"
}

void InitializeCANSocket();
void sendCANPacket(const CANPacket &packet);

// Returns 1 if a packet was received, 0 otherwise
// If a packet was received, stores it at *packet.
int recvCANPacket(CANPacket *packet);

#endif

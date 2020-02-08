
#ifndef __CAN_UTILS_H__
#define __CAN_UTILS_H__

extern "C"
{
    #include "../HindsightCAN/CANPacket.h"
}

void InitializeCANSocket();
void SendCANPacket(const CANPacket &packet);
CANPacket recvCANPacket();

#endif

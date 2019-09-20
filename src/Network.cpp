#include "Globals.h"
#include "Network.h"

#include <cassert>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

void InitializeNetwork()
{
    if(Globals::can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW))
        assert(!"Failed to initialize CAN bus!");
    
}

void ParseIncomingNetworkPackets()
{
    
}

void SendOutgoingNetworkPackets()
{
    
}

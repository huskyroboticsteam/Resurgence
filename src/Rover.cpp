#include "CommandLineOptions.h"
#include "Globals.h"
#include "Network.h"

void InitializeRover()
{
    // TODO(sasha): Call individual initialization functions
}

int main(int argc, char **argv)
{
    Globals::opts = ParseCommandLineOptions(argc, argv);
    InitializeRover();
    for(;;)
    {
        ParseIncomingNetworkPackets(); // NOTE(sasha): Since we're probably going to be using SocketCAN,
                                       //              this includes both CAN and Network packets (maybe)
        UpdateRoverState();
        SendOutgoingNetworkPackets();
    }
    return 0;
}

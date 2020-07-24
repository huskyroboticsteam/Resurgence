#include "CommandLineOptions.h"
#include "Globals.h"
#include "Networking/NetworkConstants.h"
#include "Networking/Network.h"
#include "Networking/CANUtils.h"
#include "Networking/ParseCAN.h"
#include "Networking/ParseBaseStation.h"
#include "Autonomous.h"
#include "simulator/world_interface.h"

void InitializeRover()
{
    InitializeCANSocket();
    InitializeBaseStationSocket();
}

int main(int argc, char **argv)
{
    world_interface_init();
    Globals::opts = ParseCommandLineOptions(argc, argv);
    InitializeRover();
    CANPacket packet;
    // Target location for autonomous navigation
    // Eventually this will be set by communcation from the base station
    PointXY target { 3.14, 2.71 };
    Autonomous autonomous(target);
    char buffer[MAXLINE];
    for(;;)
    {
        if (recvCANPacket(&packet) != 0) {
            ParseCANPacket(packet);
        }
        bzero(buffer, sizeof(buffer));
        if (recvBaseStationPacket(buffer) != 0) {
            ParseBaseStationPacket(buffer);
        }
        autonomous.autonomyIter();
    }
    return 0;
}

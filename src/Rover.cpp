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

const double CONTROL_HZ = 10.0;

int main(int argc, char **argv)
{
    world_interface_init();
    Globals::opts = ParseCommandLineOptions(argc, argv);
    InitializeRover();
    CANPacket packet;
    // Target location for autonomous navigation
    // Eventually this will be set by communcation from the base station
    int urc_leg = 0;
    Autonomous autonomous(getLeg(urc_leg));
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
        usleep(1000 * 1000 / CONTROL_HZ);
    }
    return 0;
}

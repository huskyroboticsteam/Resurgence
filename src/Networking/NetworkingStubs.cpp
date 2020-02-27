
#include "NetworkingStubs.h"
#include <queue>

std::queue<CANPacket> can_packets {};
std::queue<std::string> base_station_packets {};

void SendCANPacket(const CANPacket &packet)
{
  can_packets.push(packet);
}

void sendBaseStationPacket(const std::string &packet)
{
  base_station_packets.push(packet);
}

CANPacket popCANPacket()
{
  CANPacket p = can_packets.front();
  can_packets.pop();
  return p;
}

std::string popBaseStationPacket()
{
  std::string p = base_station_packets.front();
  base_station_packets.pop();
  return p;
}

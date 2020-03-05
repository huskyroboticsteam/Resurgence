
#include "NetworkingStubs.h"
#include <queue>

std::queue<CANPacket> can_packets {};
std::queue<std::string> base_station_packets {};

int numCANPackets()
{
  return can_packets.size();
}

int numBaseStationPackets()
{
  return base_station_packets.size();
}

void sendCANPacket(const CANPacket &packet)
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

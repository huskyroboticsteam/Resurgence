#include <stdio.h>
#include <assert.h>
#include "CANPacket.h"

void testBasicPacketCreation();
void testBasicCANIDConstruction();

int main() 
{
    testBasicCANIDConstruction();
    testBasicPacketCreation();     
}

void testBasicCANIDConstruction() 
{
    uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_MOTOR_CONTROL, 0x05);
    uint16_t EXPECTED = 0x0505;
    assert(CAN_ID == EXPECTED);
}

void testBasicPacketCreation() 
{
    uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_MOTOR_CONTROL, 0x05);
    uint8_t testDataPacket[2];
    testDataPacket[0] = 0x60;
    testDataPacket[1] = 0x43;
    CANPacket packet = ConstructCANPacket(CAN_ID, 0x02, testDataPacket);

    assert(packet.id == 0x0505);
    assert(packet.dlc == 0x02);
    assert(packet.data[0] == 0x60);
    assert(packet.data[1] == 0x43);
}
#include "Globals.h"
#include "Network.h"
#include "Networking/ParseCAN.h"
extern "C" 
{
    #include "HindsightCAN/Port.h"
}
#include <stdio.h>
#include <assert.h>
#include <iostream>

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

void testParseCAN() {
    uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_JETSON, DEVICE_SERIAL_JETSON);
    uint8_t testDataPacket[7];
    WriteSenderSerialAndPacketID(testDataPacket, 0x36); // 0x36 is telemetry report
    testDataPacket[2] = 0x01; // current
    testDataPacket[3] = 0x00;
    testDataPacket[4] = 0x00;
    testDataPacket[5] = 0x01; // 256 milliamps
    testDataPacket[6] = 0x00;
    CANPacket packet = ConstructCANPacket(CAN_ID, 0x07, testDataPacket);

    ParseCANPacket(packet);
    std::cout << Globals::status_data << std::endl;
    assert(Globals::status_data.dump() == "{\"front_right_motor\":{\"current\":256}}");
}

int main() 
{
    testBasicCANIDConstruction();
    std::cout << "pass\n";
    testBasicPacketCreation();
    std::cout << "pass\n";
    testParseCAN();
    std::cout << "pass\n";
}

int SendCANPacket(CANPacket packet)
{
    return 0;
}

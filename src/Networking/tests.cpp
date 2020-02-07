#include "../Globals.h"
#include "ParseCAN.h"
extern "C"
{
    #include "../HindsightCAN/Port.h"
}
#include <catch2/catch.hpp>
#include <iostream>

TEST_CASE("Basic CAN ID construction", "[CAN]")
{
    uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_MOTOR_CONTROL, 0x05);
    uint16_t EXPECTED = 0x0505;
    REQUIRE(CAN_ID == EXPECTED);
}

TEST_CASE("Basic packet creation", "[CAN]")
{
    uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_MOTOR_CONTROL, 0x05);
    uint8_t testDataPacket[2];
    testDataPacket[0] = 0x60;
    testDataPacket[1] = 0x43;
    CANPacket packet = ConstructCANPacket(CAN_ID, 0x02, testDataPacket);

    REQUIRE(packet.id == 0x0505);
    REQUIRE(packet.dlc == 0x02);
    REQUIRE(packet.data[0] == 0x60);
    REQUIRE(packet.data[1] == 0x43);
}

TEST_CASE("ParseCAN can handle telemetry", "[CAN]")
{
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
    REQUIRE(Globals::status_data.dump() == "{\"front_right_motor\":{\"current\":256}}");
}

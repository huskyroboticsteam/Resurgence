
extern "C"
{
    #include "../HindsightCAN/Port.h"
}

CANPacket motorTelemetry()
{
  uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_JETSON, DEVICE_SERIAL_JETSON);
  uint8_t testDataPacket[7];
  WriteSenderSerialAndPacketID(testDataPacket, 0x36); // 0x36 is telemetry report
  testDataPacket[2] = 0x01; // current
  testDataPacket[3] = 0x00;
  testDataPacket[4] = 0x00;
  testDataPacket[5] = 0x01; // 256 milliamps
  testDataPacket[6] = 0x00;

  return ConstructCANPacket(CAN_ID, 0x07, testDataPacket);
}

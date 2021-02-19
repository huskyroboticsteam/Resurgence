
extern "C"
{
    #include "../HindsightCAN/Port.h"
    #include "../HindsightCAN/CANCommon.h"
}

CANPacket motorTelemetry()
{
  uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_JETSON, DEVICE_SERIAL_JETSON);
  uint8_t testDataPacket[DLC_TELEMETRY_REPORT];
  WriteSenderSerialAndPacketID(testDataPacket, ID_TELEMETRY_REPORT);
  testDataPacket[3] = 0x01; // current
  testDataPacket[4] = 0x00;
  testDataPacket[5] = 0x00;
  testDataPacket[6] = 0x01; // 256 milliamps
  testDataPacket[7] = 0x00;

  return ConstructCANPacket(CAN_ID, DLC_TELEMETRY_REPORT, testDataPacket);
}

CANPacket LEDTest()
{
  uint16_t CAN_ID = ConstructCANID(PACKET_PRIORITY_NORMAL, DEVICE_GROUP_GPIO_BOARDS, DEVICE_SERIAL_JETSON); //CHANGE SERIAL - TALK TO ELECTRONICS
  uint8_t testDataPacket[DLC_LED_COLOR];
  WritePacketIDOnly(testDataPacket, ID_LED_COLOR);
  testDataPacket[1] = 0xFF; // R value
  testDataPacket[2] = 0x00; // G value
  testDataPacket[3] = 0x00; // B value

  return ConstructCANPacket(CAN_ID, DLC_LED_COLOR, testDataPacket);
}

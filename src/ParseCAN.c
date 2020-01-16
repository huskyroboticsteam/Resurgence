#include "Globals.h"
#include "Network.h"

void ParseCANPacket(CANPacket p)
{
    // Future calls for parsing data example
    // Some functions not present yet so it wont compile
    // Refer to ParseCAN.h for values needed to be parsed
    
    if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_CHASSIS_FL) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.front_left_motor_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.front_left_motor_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.front_left_motor_temp_mc = DecodeTelemetryDataSigned(p);
    }
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_CHASSIS_FR) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.front_right_motor_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.front_right_motor_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.front_right_motor_temp_mc = DecodeTelemetryDataSigned(p);
    }
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_CHASSIS_BL) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.back_left_motor_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.back_left_motor_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.back_left_motor_temp_mc = DecodeTelemetryDataSigned(p);
    }
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_CHASSIS_BR) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.back_right_motor_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.back_right_motor_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.back_right_motor_temp_mc = DecodeTelemetryDataSigned(p);
    }

    
    //TODO: Add functions for arm motor boards as well

    //TODO: Add in arm limit switch status
    
    //TODO: Add in GPS status data

    //TODO: Add in accelerometer telemetry status data

    //TODO: Add in gyroscope telemetry status data
}
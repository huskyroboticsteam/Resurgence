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

    
    //base
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_BASE) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.base_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.base_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            	StatusData Globals::status_data.base_temp_mc = DecodeTelemetryDataSigned(p);
    }

    //shoulder
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_SHOULDER) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.shoulder_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.shoulder_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.shoulder_temp_mc = DecodeTelemetryDataSigned(p);
    }

    //elbow
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_ELBOW) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.elbow_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.elbow_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.elbow_temp_mc = DecodeTelemetryDataSigned(p);
    }

    //forearm/wrist
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_FOREARM) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.wrist_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.wrist_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.wrist_temp_mc = DecodeTelemetryDataSigned(p);
    }


    //diffleft/wrist1
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_DIFF_WRIST_1) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.diffleft_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.diffleft_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.diffleft_temp_mc = DecodeTelemetryDataSigned(p);
    }

    //diffright/wrist2
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_DIFF_WRIST_2) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.diffright_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.diffright_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.diffright_temp_mc = DecodeTelemetryDataSigned(p);
    }

    //hand
    else if (SenderPacketIsInGroup(p, DEVICE_GROUP_MOTOR_CONTROL) &&
            SenderPacketIsOfDevice(p, DEVICE_SERIAL_MOTOR_HAND) &&
            PacketIsOfID(p, ID_TELEMETRY_REPORT))
    {
        if (DecodeTelemetryType(p) == PACKET_TELEMETRY_VOLTAGE)
            StatusData Globals::status_data.hand_voltage_mv = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_CURRENT)
            StatusData Globals::status_data.hand_current_ma = DecodeTelemetryDataSigned(p);
        else if (DecodeTelemetryType(p) == PACKET_TELEMETRY_TEMPERATURE)
            StatusData Globals::status_data.hand_temp_mc = DecodeTelemetryDataSigned(p);
    }

    //TODO: Add in arm limit switch status
    
    //TODO: Add in GPS status data

    //TODO: Add in accelerometer telemetry status data

    //TODO: Add in gyroscope telemetry status data
}

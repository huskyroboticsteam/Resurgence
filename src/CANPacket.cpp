#include "CANPacket.h"
// Contains functons for creating CAN packets
// Functions here will be used in Jetson (Rover.cpp) and electronics boards

// Constructs a CAN ID according to standards set by electronics subsystem
// for PY2020 rover. Not compatible with Orpheus (PY2019)
// Inputs: 
//      priority:   A boolean determing if the packet should be prioritized
//                  Priority of true means bit is zero and vise versa
//      devGroup:   ID of device group
//      devSerial:  Serial value of device in the device group
// Output:
//      CANID:      CAN ID with correct formatting        
uint16_t ConstructCANID(bool priority, uint8_t devGroup, uint8_t devSerial)
{
    uint16_t CANID = 0;
    CANID = CANID & (0x2FF << (priority & 0x1));
    CANID = CANID & (0x2F << (devGroup & 0xF));
    CANID = CANID & (devSerial & 0x2F);

    return CANID;
}

// Creates a CANPacket that can be used by fuctions in this file
// Inputs:
//      id:         CAN ID for the packet
//      dlc:        Data length for the packet. It's the number of bytes used
//                  in data payload for packet
//      data:       An array of bytes used for sending data over CAN
// Outputs:
//      CANPacket:  A struct used for storing the parts needed for a CAN Packet
CANPacket ConstructCANPacket(uint16_t id, uint8_t dlc, uint8_t data[8])
{
    CANPacket cp;
    cp.id = id;
    cp.dlc = dlc;   
    for(int i = 0; i < 8; i++){
        cp.data[i] = data[i];
    }

}
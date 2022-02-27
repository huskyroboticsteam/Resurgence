#pragma once

#include "CAN.h"

namespace can {
class CANDevice {
public:
	CANDevice(devicegroup_t group, deviceserial_t serial);
	int32_t getTelemetry(uint8_t telemType);

protected:
	devicegroup_t group;
	deviceserial_t serial;
};

class CANMotor : public CANDevice {
public:
	CANMotor(deviceserial_t serial);
	int32_t getEncoderPosition();
	void setEncoderPosition();
	void setPower(double power);
	void setPIDTarget(int32_t pos);
	bool supportsPID();
};
} // namespace can
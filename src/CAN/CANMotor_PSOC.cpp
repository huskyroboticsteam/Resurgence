#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

extern "C" {
#include "../HindsightCAN/CANCommon.h"
#include "../HindsightCAN/CANMotorUnit.h"
#include "../HindsightCAN/CANPacket.h"
}

using namespace std::chrono_literals;
using std::chrono::milliseconds;

namespace can::motor {

namespace {
using clock = std::chrono::steady_clock;
using timepoint_t = std::chrono::time_point<clock>;

struct telemschedule_t {
	timepoint_t nextSendTime;
	milliseconds telemPeriod;
	deviceserial_t serial;
};

bool operator<(const telemschedule_t& t1, const telemschedule_t& t2) {
	return t1.nextSendTime < t2.nextSendTime;
}

std::priority_queue<telemschedule_t> telemetrySchedule;
bool startedMotorThread = false;
bool newMotorAdded = false;
std::condition_variable motorsCV;
std::mutex motorsMutex; // protects telemetrySchedule, startedMotorThread, newMotorAdded

void motorThreadFn() {
	while (true) {
		std::unique_lock lock(motorsMutex);
		if (telemetrySchedule.empty()) {
			motorsCV.wait(lock, [] { return newMotorAdded; });
			newMotorAdded = false;
		} else {
			auto now = clock::now();
			auto ts = telemetrySchedule.top();
			if (ts.nextSendTime <= now) {
				telemetrySchedule.pop();
				pullMotorPosition(ts.serial);
				telemetrySchedule.push(
					{ts.nextSendTime + ts.telemPeriod, ts.telemPeriod, ts.serial});
			} else {
				motorsCV.wait_until(lock, ts.nextSendTime, [] { return newMotorAdded; });
				newMotorAdded = false;
			}
		}
	}
}

void startMonitoringMotor(deviceserial_t motor, std::chrono::milliseconds period) {
	{
		std::unique_lock lock(motorsMutex);
		if (!startedMotorThread) {
			startedMotorThread = true;
			std::thread motorThread(motorThreadFn);
			motorThread.detach();
		}
		telemetrySchedule.push({clock::now(), period, motor});
		newMotorAdded = true;
	}
	motorsCV.notify_one();
}
} // namespace

void initEncoder(deviceserial_t serial, bool invertEncoder, bool zeroEncoder,
				 int32_t pulsesPerJointRev, std::optional<std::chrono::milliseconds> telemetryPeriod) {
	auto motorGroupCode = static_cast<uint8_t>(devicegroup_t::motor);
	CANPacket p;
	AssembleEncoderInitializePacket(&p, motorGroupCode, serial, sensor_t::encoder,
									invertEncoder, zeroEncoder);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	AssembleEncoderPPJRSetPacket(&p, motorGroupCode, serial, pulsesPerJointRev);
	sendCANPacket(p);
	std::this_thread::sleep_for(1000us);
	if (telemetryPeriod) {
		startMonitoringMotor(serial, telemetryPeriod.value());
	}
}

void pullMotorPosition(deviceserial_t serial) {
	CANPacket p;
	AssembleTelemetryPullPacket(&p, static_cast<uint8_t>(devicegroup_t::motor), serial,
								static_cast<uint8_t>(telemtype_t::angle));
	sendCANPacket(p);
}
} // namespace can::motor

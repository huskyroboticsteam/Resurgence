#include "can_motor.h"
#include "../world_interface/real_world_constants.h"
#include "CAN.h"
#include "CANMotor.h"
#include "CANUtils.h"

namespace robot {
class can_motor: public base_motor {
	public: 
		can_motor(robot::types::motorid_t motor, bool hasPosSensor) {
			motor_id = motor;
			posSensor = hasPosSensor;
		}

		bool hasPositionSensor() const {
			return posSensor;
		}

		void setMotorPower(double power) {
			can::deviceserial_t serial = motorSerialIDMap.at(motor_id);
			auto& scaleMap = power < 0 ? negative_pwm_scales : positive_pwm_scales;
			auto entry = scaleMap.find(motor_id);
			if (entry != scaleMap.end()) {
				power *= entry->second;
			}
			can::motor::setMotorPower(serial, power);
		}

		void setMotorPos(int32_t targetPos) {
			can::deviceserial_t serial = motorSerialIDMap.at(motor_id);
			can::motor::setMotorPIDTarget(serial, targetPos);
		}

		types::DataPoint<int32_t> getMotorPos() const {
			return can::motor::getMotorPosition(motorSerialIDMap.at(motor_id));
		}

		void setMotorVel(int32_t targetVel) {
			// thread will run when called for the first time (std::thread in global) / velocity thread
			// if no thread running, spin up a thread
			// velocity task (Evan wrote this a while ago)
			// at fixed update rate: iterate over motors, 

			// main pitfalls: setMotorPower may be controlled by velocity task or other tasks and function is in another place
				// how do we know if it's being controlled by a task?

			// make motor an object
				// getMotor() in world interface
				// shared ptr for polymorphism
				// different implementation depending on world interface (init method in world interfaces - sim motor, can motor)

			// get motor position and set its dimensions
			types::DataPoint<int32_t> motorPos = getMotorPos();
			if (!motorPos.isValid()) {
				return;
			}
			constexpr int32_t inputDim = 1;
			constexpr int32_t outputDim = 1;

			// create vectors for motor position and velocity
			navtypes::Vectord<inputDim> posVector {motorPos.getData()};
			navtypes::Vectord<inputDim> velocityVector {targetVel};

			// create kinematics function (input and output will both be the current motor position)
			const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>& kinematicsFunct = 
				[](const navtypes::Vectord<inputDim>& inputVec)->navtypes::Vectord<outputDim> { 
				// returns a copy of the input vector
				navtypes::Vectord<outputDim> res {inputVec(0)};
				return res; }
			;

			// create jacobian function (value will be 1 since it's the derivative of the kinematics function)
			const std::function<navtypes::Matrixd<outputDim, inputDim>(const navtypes::Vectord<inputDim>&)>& jacobianFunct = 
				[](const navtypes::Vectord<inputDim>& inputVec)->navtypes::Matrixd<outputDim, inputDim> { 
					navtypes::Matrixd<outputDim, inputDim> res = navtypes::Matrixd<outputDim, inputDim>::Ones();
				}
			;

			// get the current time
			types::datatime_t currTime = types::dataclock::now();

			// set the target velocity
			JacobianVelController<outputDim, inputDim> velController(kinematicsFunct, jacobianFunct);
			velController.setTarget(currTime, velocityVector);
			assert(("Target velocity should be set in setMotorVel", velController.hasTarget()));
		}

}; // class can_motor
} // namespace robot
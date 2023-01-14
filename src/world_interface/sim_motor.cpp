#include "sim_motor.h"
#include "../Globals.h"
#include "world_interface.h"

namespace robot {
class sim_motor: public base_motor {
	public:
		sim_motor(robot::types::motorid_t motor, bool hasPosSensor, std::string name, std::string path) {
			motor_id = motor;
			posSensor = hasPosSensor;
            motor_name = name;
            protocol_path = path;
		}

		bool hasPositionSensor() const {
			return posSensor;
		}

		void setMotorPower(double power) {
	        json msg = {{"type", "simMotorPowerRequest"}, {"motor", motor_name}, {"power", power}};
			sendJSON(msg);
		}

		void setMotorPos(int32_t targetPos) {
	        json msg = {{"type", "simMotorPositionRequest"}, {"motor", motor_name}, {"position", targetPos}};
            sendJSON(msg);
		}

        types::DataPoint<int32_t> getMotorPos() const {
			robot::getMotorPos(motor_id);
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
    
    private:
        std::string motor_name;
        std::string protocol_path;

        void sendJSON(const json& obj) {
	        Globals::websocketServer.sendJSON(protocol_path, obj);
        }
}; // class sim_motor
} // namespace robot
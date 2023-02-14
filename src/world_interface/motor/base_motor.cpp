#include "base_motor.h"

namespace robot {
    bool base_motor::hasPositionSensor() const {
		return posSensor;
	}

    void base_motor::setMotorVel(int32_t targetVel) {
        // create a velocity controller if it's not already initialized
        if (!velController) {
            constructVelController();
        }

        // set velocity target
        navtypes::Vectord<1> velocityVector{targetVel};
        types::datatime_t currTime = types::dataclock::now();
        velController->setTarget(currTime, velocityVector);

        // check to see if the event exists. if yes, unschedule it
        base_motor::resetEventID();

        // schedule position event
        velEventID = pSched->scheduleEvent(100ms, [&]() -> void {
            types::datatime_t currTime = types::dataclock::now();
            const navtypes::Vectord<1> currPos{getMotorPos().getData()};
            navtypes::Vectord<1> posCommand = velController->getCommand(currTime, currPos);
            // TODO: is there only one position value in the matrix?
            setMotorPos(posCommand.coeff(0, 0));
        });
    }

    void base_motor::constructVelController() {
		types::DataPoint<int32_t> motorPos = getMotorPos();
		if (!motorPos.isValid()) {
			return;
		}
		constexpr int32_t inputDim = 1;
		constexpr int32_t outputDim = 1;

		// create vector for motor position
		navtypes::Vectord<inputDim> posVector{motorPos.getData()};

		// create kinematics function (input and output will both be the current motor
		// position)
		const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>& kinematicsFunct = 
            [](const navtypes::Vectord<inputDim>& inputVec) { 
                // returns a copy of the input vector
                navtypes::Vectord<outputDim> res {inputVec(0)};
                return res; 
            };

        // create jacobian function (value will be 1 since it's the derivative of the 
        // kinematics function)
        const std::function<navtypes::Matrixd<outputDim, inputDim>(const navtypes::Vectord<inputDim>&)>& jacobianFunct = 
            [](const navtypes::Vectord<inputDim>& inputVec) { 
                navtypes::Matrixd<outputDim, inputDim> res = navtypes::Matrixd<outputDim, inputDim>::Ones();
                return res;
            };

		velController.emplace(kinematicsFunct, jacobianFunct);
	}

    void base_motor::resetEventID() {
		if (velEventID) {
			pSched->removeEvent(velEventID.value());
			velEventID.reset();
		}
	}
} // namespace robot
#pragma once

#include "data.h"
#include "../control/JacobianVelController.h"
#include "../utils/scheduler.h"

using namespace std::chrono_literals;

/**
 * @namespace robot
 * @brief Collection of functions for manipulating a motor.
 */
namespace robot {
/**
 * An abstract motor class
 * 
 */
class base_motor {
public:

    // TODO: spin up thread in constructor, use it for setMotorVel (static mutex lock)

    /**
     * @brief Default constructor for base motor.
     *
     */
    base_motor();

    /**
     * @brief Returns the status of the motor position sensor.
     *
     * @return True if the motor has a position sensor and false if not
     */
    virtual bool hasPositionSensor() const = 0;

    /**
     * @brief Set the PWM command of the motor.
     *
     * @param power The power command, in the range [-1, 1]
     */
    virtual void setMotorPower(double power) = 0;

    /**
     * @brief Set the target position of the motor. This will have no effect if the motor
     * does not support PID.
     *
     * @param targetPos The target position, in millidegrees. Refer to the specific motor for more
     * information.
     */
    virtual void setMotorPos(int32_t targetPos) = 0;

    /**
     * @brief Get the last reported position of the specified motor.
     *
     * @return types::DataPoint<int32_t> The last reported position of the motor, if it exists.
     * If the motor has not reported a position (because it hasn't been received yet or if it
     * doesn't have an encoder) then an empty data point is returned.
     */
    virtual types::DataPoint<int32_t> getMotorPos() const = 0;

    /**
     * @brief Sets the velocity of the motor.
     *
     * @param targetVel The target velocity, in millidegrees per second.
     */
    void setMotorVel(int32_t targetVel) {
        // create a velocity controller if it's not already initialized
        if (!velController) {
            constructVelController();
        }
        assert(("Vel controller is initialized", velController));

        // set velocity target
        navtypes::Vectord<1> velocityVector {targetVel};
        types::datatime_t currTime = types::dataclock::now();
        velController->setTarget(currTime, velocityVector);
        assert(("Target velocity should be set in setMotorVel", velController->hasTarget()));

        // check to see if the event exists. if yes, unschedule it
        resetEventID();

        // schedule position event
        velEventID = pSched.scheduleEvent(100ms, [&]() -> void {
                        types::datatime_t currTime = types::dataclock::now();
                        const navtypes::Vectord<1> currPos {getMotorPos().getData()};
                        navtypes::Vectord<1> posCommand = velController->getCommand(currTime, currPos);
                        // TODO: is there only one position value in the matrix?
                        setMotorPos(posCommand.coeff(0, 0));
                    });
    }

protected:
    robot::types::motorid_t motor_id;
    bool posSensor;
    static util::PeriodicScheduler<std::chrono::steady_clock> pSched;
    std::optional<util::PeriodicScheduler<std::chrono::steady_clock>::eventid_t> velEventID;
    std::optional<JacobianVelController<1, 1>> velController;

    /**
     * @brief Constructs a Jacobian Vel Controller
     *
     */
    void constructVelController() {
        types::DataPoint<int32_t> motorPos = getMotorPos();
        if (!motorPos.isValid()) {
            return;
        }
        constexpr int32_t inputDim = 1;
        constexpr int32_t outputDim = 1;

        // create vector for motor position
        navtypes::Vectord<inputDim> posVector {motorPos.getData()};

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

        velController = {kinematicsFunct, jacobianFunct};
    }

    /**
     * @brief Unschedules the velocity event if it exists
     *
     */
    void resetEventID() {
        if (velEventID) {
            pSched.removeEvent(velEventID.value());
            velEventID.reset();
        }
        assert(("velEventId has been removed", !velEventID));
    }
}; // abstract class base_motor
} // namespace robot
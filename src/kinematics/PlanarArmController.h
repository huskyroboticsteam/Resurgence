#pragma once

#include "../navtypes.h"
#include "PlanarArmKinematics.h"

#include <array>
#include <initializer_list>
#include <numeric>

#include <Eigen/Core>

namespace kinematics {

/**
 * @brief Controller to move planar arm to target position.
 *
 * TODO: GOAL is to move the arm to the target positon (which is the end effector position)
 * Input: target / setpoint (x, y), stuff to make planararmkinematics
 *
 * Steps:
 *  Get current arm joint positions
 *  Use PlanarArmKinematics to get the joint angles and velocities that yield the desired EE
 * position (setpoint) Get the command for moving the arm (bound setpoint target) Return the
 * command
 *
 * Variables:
 *  PlanarArmKinematics
 *  Setpoint (x, y)
 *  Velocities (x, y)
 *
 * Methods:
 *  Constructor
 *  For returning the current command
 *  For changing x and y velocities
 *
 * @tparam N The number of joints.
 */
template <unsigned int N> class PlanarArmController {
public:
	/**
	 * @brief Construct a new controller object.
	 *
	 * @param target Setpoint for end effector {double x, double y}
	 * @param kin_obj PlanarArmKinematics object for the arm
	 */
	PlanarArmController(Eigen::Vector2d target PlanarArmKinematics<N> kin_obj)
		: setpoint(target), kinematics(kin_obj) {}

	void set_x_vel(robot::types::datatime_t currTime, int32_t targetVel, const navtypes::Vectord<N>& currPose) {
		timestamp = currTime;
		velocity(0) = targetVel;
		getCommand(currTime, currPose);
	}

	void set_y_vel(robot::types::datatime_t currTime, int32_t targetVel, const navtypes::Vectord<N>& currPose) {
		timestamp = currTime;
		velocity(1) = targetVel;
		getCommand(currTime, currPose);
	}

	navtypes::Vectord<N> getCommand(robot::types::datatime_t currTime,
									const navtypes::Vectord<N>& currPose) {
		// create kinematics function
		const std::function<navtypes::Vectord<outputDim>(const navtypes::Vectord<inputDim>&)>&
			kinematicsFunc = [](const navtypes::Vectord<inputDim>& inputVec) {
				// returns a copy of the input vector
				return inputVec;
			};

		// get jacobian function
		navtypes::Matrixd<2, N> jacobianFunc = kinematics.getJacobian(currPose);

		// get target joint angles
        navtypes::Vectord<N> currTarget = kinematics.eePosToJointPos(setpoint, currPose)

		// get joint velocity
        navtypes::Vectord<N> jointVels = kinematics.eeVelToJointVel(currPose, velocity)

        // define dimensions
	    constexpr int32_t inputDim = N;
	    constexpr int32_t outputDim = N;

		// calculate command
		navtypes::Vectord<outputDim> currPosOutput = kinematicsFunc(currPos);
		navtypes::Vectord<outputDim> outputPosDiff = currTarget - currPosOutput;
		navtypes::Matrixd<outputDim, inputDim> jacobian = jacobianFunc(currPos);
		navtypes::Vectord<inputDim> inputPosDiff =
			jacobian.colPivHouseholderQr().solve(outputPosDiff);

		return currPose + inputPosDiff * jointVels;
	}

private:
	Eigen::Vector2d setpoint;
	Eigen::Vector2d velocity{0.0, 0.0};
	PlanarArmKinematics<N> kinematics;
	robot::types::datatime_t timestamp;
};
} // namespace kinematics
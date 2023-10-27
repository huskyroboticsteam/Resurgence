#pragma once

#include "../utils/time.h"
#include "../world_interface/data.h"

#include <chrono>
#include <functional>
#include <optional>
#include <utility>

#include <Eigen/Core>

namespace control {

/**
 * @brief A set of PID gains.
 * @see https://en.wikipedia.org/wiki/PID_controller
 */
struct PIDGains {
	double kP, kI, kD, kF;
	std::optional<double> iZone;
};

/**
 * @brief A PID controller for multiple independent dimensions.
 *
 * @tparam dim The number of dimensions to control.
 * @see https://en.wikipedia.org/wiki/PID_controller
 */
template <int dim>
class PIDController {
public:
	template <int d>
	using Arrayd = Eigen::Array<double, d, 1>;

	/**
	 * @brief Construct a new PIDController.
	 *
	 * @param gains The gains to use for the controller.
	 */
	PIDController(const std::array<PIDGains, dim>& gains) {
		reset();
		setGains(gains);
	}

	/**
	 * @brief Set the gains of the PID controller.
	 *
	 * @param gains The new gains to set.
	 */
	void setGains(const std::array<PIDGains, dim>& gains) {
		for (int i = 0; i < dim; i++) {
			kP(i) = gains[i].kP;
			kI(i) = gains[i].kI;
			kD(i) = gains[i].kD;
			kF(i) = gains[i].kF;
			iZone(i) = gains[i].iZone.value_or(std::numeric_limits<double>::infinity());
		}
	}

	/**
	 * @brief Check if a target is currently set.
	 *
	 * @return bool True iff the controller currently has a target.
	 */
	bool hasTarget() {
		return setPoint.has_value();
	}

	/**
	 * @brief Set the target of the controller.
	 *
	 * @param setPoint An array of setpoints, one for each dimension.
	 */
	void setTarget(const Arrayd<dim>& setPoint) {
		this->setPoint = setPoint;
	}

	/**
	 * @brief Get the output from the controller.
	 *
	 * @param currTime The current timestamp.
	 * @param currValue The current values of the controlled signals.
	 * @return Arrayd<dim> Targets for each dimension if target is set, else zeros.
	 */
	Arrayd<dim> getOutput(robot::types::datatime_t currTime, const Arrayd<dim>& currValue) {
		if (!setPoint.has_value()) {
			return Arrayd<dim>::Zero();
		}

		// see wikipedia link above for in-depth explanation

		Arrayd<dim> err = setPoint.value() - currValue;
		Arrayd<dim> pTerm = err * kP;
		Arrayd<dim> fTerm = setPoint.value() * kF;

		Arrayd<dim> iTerm = Arrayd<dim>::Zero();
		Arrayd<dim> dTerm = Arrayd<dim>::Zero();

		if (lastData.has_value()) {
			// TODO: instead of numerically differentiating we can use a model (if we have one)
			auto [lastTime, lastValue] = lastData.value();
			double dtSec = util::durationToSec(currTime - lastTime);

			// use the backwards finite difference for approximating the derivative
			Arrayd<dim> derivative = (currValue - lastValue) / dtSec;
			dTerm = derivative * kD;

			// use the trapezoid rule for approximating the integral
			Arrayd<dim> area = (lastValue + currValue) * dtSec / 2.0;
			iAccum += area;
			iTerm = (err.abs() <= iZone).select(iAccum * kI, Arrayd<dim>::Zero());
		}

		lastData = {currTime, currValue};
		Arrayd<dim> output = pTerm + iTerm + dTerm + fTerm;

		return output;
	}

	/**
	 * @brief Reset the controller.
	 */
	void reset() {
		setPoint.reset();
		lastData.reset();
		iAccum.setZero();
	}

private:
	Arrayd<dim> kP, kI, kD, kF;
	Arrayd<dim> iZone; // +inf indicates no i zone
	Arrayd<dim> iAccum;
	std::optional<Arrayd<dim>> setPoint;
	std::optional<std::pair<robot::types::datatime_t, Arrayd<dim>>> lastData;
};
} // namespace control

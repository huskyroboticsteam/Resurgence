#pragma once

#include "../world_interface/data.h"

#include <chrono>
#include <functional>
#include <optional>
#include <utility>

#include <Eigen/Core>

namespace control {

constexpr int MS_PER_S = 1000;

struct PIDGains {
	double kP, kI, kD, kF;
	double iZone;
};

template <int dim> class PIDController {
public:
	template <int d> using Arrayd = Eigen::Array<double, d, 1>;

	PIDController(const std::array<PIDGains, dim>& gains)
		: PIDController(
			  gains, [](const auto& x) { return x; }, [](const auto& x) { return x; }) {}

	PIDController(
		const std::array<PIDGains, dim>& gains,
		std::function<Arrayd<dim>(const Arrayd<dim>&)> ctrlToObsFn,
		std::function<Eigen::Matrix<double, dim, dim>(const Arrayd<dim>&)> ctrlToObsJacobianFn)
		: ctrlToObsFn(ctrlToObsFn), ctrlToObsJacobianFn(ctrlToObsJacobianFn) {
		reset();
		setGains(gains);
	}

	void setGains(const std::array<PIDGains, dim>& gains) {
		for (int i = 0; i < dim; i++) {
			kP(i) = gains[i].kP;
			kI(i) = gains[i].kI;
			kD(i) = gains[i].kD;
			kF(i) = gains[i].kF;
			iZone(i) = gains[i].iZone;
		}
	}

	bool hasTarget() {
		return setPoint.has_value();
	}

	void setTarget(const Arrayd<dim>& setPoint) {
		this->setPoint = ctrlToObsJacobianFn(setPoint);
	}

	Arrayd<dim> getOutput(robot::types::datatime_t currTime, const Arrayd<dim>& currValue_) {
		if (!setPoint.has_value()) {
			return Arrayd<dim>::Zero();
		}

		Arrayd<dim> currValue = ctrlToObsFn(currValue_);

		Arrayd<dim> err = setPoint.value() - currValue;
		Arrayd<dim> pTerm = err * kP;
		Arrayd<dim> fTerm = setPoint.value() * kF;

		Arrayd<dim> iTerm = Arrayd<dim>::Zero();
		Arrayd<dim> dTerm = Arrayd<dim>::Zero();

		if (lastData.has_value()) {
			auto [lastTime, lastValue] = lastData.value();
			auto dt =
				std::chrono::duration_cast<std::chrono::milliseconds>(currTime - lastTime);
			double dtSec = dt.count() / static_cast<double>(MS_PER_S);

			Arrayd<dim> derivative = (currValue - lastValue) / dtSec;
			dTerm = derivative * kD;

			Arrayd<dim> area = (lastValue + currValue) * dtSec / 2.0;
			iAccum += area;
			iTerm = (err.abs() <= iZone).select(iAccum * kI, Arrayd<dim>::Zero());
		}

		lastData = {currTime, currValue};
		Arrayd<dim> output = pTerm + iTerm + dTerm + fTerm;

		// TODO: verify - is the inverse jacobian the right thing to do here?
		// It's not. Use jacobian of position, or just use ctrlToObsPowerMapping or something.
		// Controlled value doesn't have to be position.
		Eigen::Matrix<double, dim, dim> jacobian = ctrlToObsJacobianFn(currValue_);
		return jacobian.colPivHouseholderQR().solve(output.matrix());
	}

	void reset() {
		setPoint.reset();
		lastData.reset();
		iAccum.setZero();
	}

private:
	Arrayd<dim> kP, kI, kD, kF;
	Arrayd<dim> iZone;
	Arrayd<dim> iAccum;
	std::optional<Arrayd<dim>> setPoint;
	std::optional<std::pair<robot::types::datatime_t, Arrayd<dim>>> lastData;

	// trf takes from observed space -> control space, trvInv is inverse
	std::function<Arrayd<dim>(const Arrayd<dim>&)> ctrlToObsFn;
	std::function<Eigen::Matrix<double, dim, dim>(const Arrayd<dim>&)> ctrlToObsJacobianFn;
};
} // namespace control

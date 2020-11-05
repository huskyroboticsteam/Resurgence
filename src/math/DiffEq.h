#pragma once

#include <Eigen/Core>

namespace
{
double epsilon = 1e-5;
}

template <typename T> T rungeKutta(std::function<T(const T &)> f, const T &x, double dt)
{
	// Reference:
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#The_Runge%E2%80%93Kutta_method
	double halfDt = dt / 2;
	T k1 = f(x);
	T k2 = f(x + halfDt * k1);
	T k3 = f(x + halfDt * k2);
	T k4 = f(x + dt * k3);
	return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

template <typename T, typename U>
T bifunctionRungeKutta(std::function<T(const T &, const U &)> f, const T &x, const U &u,
					   double dt)
{
	// Reference:
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#The_Runge%E2%80%93Kutta_method
	double halfDt = dt / 2;
	T k1 = f(x, u);
	T k2 = f(x + halfDt * k1, u);
	T k3 = f(x + halfDt * k2, u);
	T k4 = f(x + dt * k3, u);
	return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

template <int size, int sizeU>
Eigen::Matrix<double, size, size> bifunctionJacobianX(
	std::function<Eigen::Matrix<double, size, 1>(const Eigen::Matrix<double, size, 1> &,
												 const Eigen::Matrix<double, sizeU, 1> &)>
		f,
	const Eigen::Matrix<double, size, 1> &x, const Eigen::Matrix<double, sizeU, 1> &u)
{
	Eigen::Matrix<double, size, size> jacobian;
	for (int i = 0; i < size; i++)
	{
		Eigen::Matrix<double, size, 1> delta = Eigen::Matrix<double, size, 1>::Zero();
		delta[i] = epsilon;
		Eigen::Matrix<double, size, 1> derivative =
			(f(x + delta, u) - f(x - delta, u)) / (2 * epsilon);
		jacobian.col(i) = derivative;
	}
	return jacobian;
}

template <int size>
Eigen::Matrix<double, size, size> unifunctionJacobianX(
	std::function<Eigen::Matrix<double, size, 1>(const Eigen::Matrix<double, size, 1> &)> f,
	const Eigen::Matrix<double, size, 1> &x)
{
	Eigen::Matrix<double, size, size> jacobian;
	for (int i = 0; i < size; i++)
	{
		Eigen::Matrix<double, size, 1> delta = Eigen::Matrix<double, size, 1>::Zero();
		delta[i] = epsilon;
		Eigen::Matrix<double, size, 1> derivative =
			(f(x + delta) - f(x - delta)) / (2 * epsilon);
		jacobian.col(i) = derivative;
	}
	return jacobian;
}

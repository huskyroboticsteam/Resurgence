
#include "graph.h"

#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

namespace filters::pose_graph {

AbstractFactor::~AbstractFactor() {}

Graph::Graph()
	: _x0(values::Zero(1)), _sol(values::Zero(1)), _sol_cov(hessian::Zero(1, 1)), _factors({}),
	  _factor_counter(0) {}

Graph::~Graph() {
	for (auto f : _factors) {
		delete f;
	}
}

int Graph::add(AbstractFactor* f) {
	f->id = _factor_counter++;
	_factors.push_back(f);
	return f->id;
}

double Graph::eval(const values& x) {
	double sum = 0.0;
	for (auto f : _factors)
		sum += f->eval(x);
	return sum;
}

void Graph::solve(const values& x0, double alpha, int maxiters, double tol) {
	_x0 = x0;
	values x = x0;
	double error = 2 * tol;
	int i = 0;
	int N = x0.size();
	while (error > tol && i < maxiters) {
		values grad = values::Zero(N);
		hessian hess = hessian::Zero(N, N);
		for (auto f : _factors) {
			grad += f->gradient_at(x);
			hess += f->hessian_at(x);
		}
		for (int j = 0; j < N; j++) {
			// Presumably we have no factors affecting this variable
			if (hess(j, j) == 0)
				hess(j, j) = 0.001; // avoid singular matrix
		}
		x -= alpha * (hess.inverse() * grad);
		error = sqrt(grad.transpose() * grad);
		i += 1;
		if (i % 100 == 0)
			std::cout << "Graph::solve Iteration " << i << ": " << error << std::endl;
	}
	_sol = x;

	hessian hess = hessian::Zero(N, N);
	for (auto f : _factors)
		hess += f->hessian_at(_sol);
	for (int j = 0; j < N; j++) {
		if (hess(j, j) == 0)
			hess(j, j) = 0.001; // Again, avoid singular matrix
	}
	_sol_cov = hess.inverse();
}

values Graph::x0() {
	return _x0;
}

values Graph::solution() {
	return _sol;
}

hessian Graph::covariance() {
	return _sol_cov;
}

void Graph::deleteFactor(int id) {
	auto it = _factors.begin();
	while (it != _factors.end()) {
		if ((*it)->id == id) {
			delete *it;
			it = _factors.erase(it);
			return;
		} else {
			++it;
		}
	}
}

void Graph::shiftIndices(int poseSize, int firstPoseIdx) {
	auto it = _factors.begin();
	while (it != _factors.end()) {
		if (!(*it)->shiftIndices(poseSize, firstPoseIdx)) {
			delete *it;
			it = _factors.erase(it);
		} else {
			++it;
		}
	}
}

} // namespace filters::pose_graph

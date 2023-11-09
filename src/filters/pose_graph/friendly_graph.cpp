
#include "friendly_graph.h"

#include "../../Constants.h"
#include "../../navtypes.h"
#include "../../utils/transform.h"

#include <iostream>
#include <loguru.hpp>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

using namespace navtypes;
using util::toPose;
using util::toTransform;
using util::toTransformRotateFirst;

namespace filters::pose_graph {

constexpr int LM_SIZE = 2;
constexpr int POSE_SIZE = 3;

FriendlyGraph::FriendlyGraph(int num_landmarks, int max_num_poses, float camera_std,
							 float gps_xy_std, float wheel_noise_rate)
	: _num_landmarks(num_landmarks), _max_pose_id(0), _min_pose_id(0),
	  _max_num_poses(max_num_poses), _current_guess(LM_SIZE * num_landmarks),
	  _landmark_factor_ids(num_landmarks), _odom_cov_inv(), _sensor_cov_inv(), _gps_cov_inv(),
	  _graph() {
	covariance<3> odom_cov = covariance<3>::Zero();
	// TODO what are the right numbers here? Should y be correlated with theta?
	odom_cov << wheel_noise_rate, 0, 0, 0, wheel_noise_rate, wheel_noise_rate / 2.0, 0,
		wheel_noise_rate / 2.0, wheel_noise_rate;
	_odom_cov_inv = odom_cov.inverse();
	covariance<2> sensor_cov = covariance<2>::Zero();
	sensor_cov << camera_std * camera_std, 0, 0, camera_std * camera_std;
	_sensor_cov_inv = sensor_cov.inverse();
	// GPS has no heading measurements
	// which we represent using a large covariance
	covariance<3> gps_cov = covariance<3>::Zero();
	gps_cov << gps_xy_std * gps_xy_std, 0, 0, 0, gps_xy_std * gps_xy_std, 0, 0, 0, 50.0 * 50.0;
	_gps_cov_inv = gps_cov.inverse();
	for (int id = 0; id < num_landmarks; id++) {
		_landmark_factor_ids[id] = -1;
	}
}

int FriendlyGraph::numPoses() {
	return _max_pose_id - _min_pose_id;
}

int FriendlyGraph::nonincrementingPoseIdx(int pose_id) {
	return _num_landmarks * LM_SIZE + (pose_id - _min_pose_id) * POSE_SIZE;
}

void FriendlyGraph::incrementNumPoses() {
	_max_pose_id++;
	_current_guess.conservativeResize(nonincrementingPoseIdx(_max_pose_id));
}

int FriendlyGraph::poseIdx(int pose_id) {
	if (pose_id == _max_pose_id) {
		incrementNumPoses();
	} else if (pose_id > _max_pose_id) {
		printf("Error: skipped a pose id (given %d, current %d)\n", pose_id, _max_pose_id);
		throw 1;
	}
	return nonincrementingPoseIdx(pose_id);
}

int FriendlyGraph::landmarkIdx(int lm_id) {
	CHECK_LT_F(lm_id, _num_landmarks);
	return lm_id * LM_SIZE;
}

void FriendlyGraph::trimToMaxNumPoses() {
	if (numPoses() > _max_num_poses) {
		if (numPoses() != _max_num_poses + 1) {
			printf("Error: skipped a trim\n");
			throw 2;
		}
		hessian sol_cov = _graph.covariance();
		int base_idx = poseIdx(_min_pose_id + 1);
		measurement<3> first_pose = getPoseEstimate(_min_pose_id + 1);
		covariance<3> first_pose_cov = sol_cov.block(base_idx, base_idx, POSE_SIZE, POSE_SIZE);
		values old_guess = _current_guess;
		// printf("Trimming. Current uncertainty on base pose: %f %f %f\n",
		//     sqrt(first_pose_cov(0,0)), sqrt(first_pose_cov(1,1)),
		//     sqrt(first_pose_cov(2,2)));
		_graph.shiftIndices(POSE_SIZE, poseIdx(_min_pose_id));
		_min_pose_id += 1;
		_current_guess.conservativeResize(nonincrementingPoseIdx(_max_pose_id));
		_current_guess.block(poseIdx(_min_pose_id), 0, _max_num_poses * POSE_SIZE, 1) =
			old_guess.block(poseIdx(_min_pose_id + 1), 0, _max_num_poses * POSE_SIZE, 1);
		addPosePrior(_min_pose_id, toTransform(first_pose), first_pose_cov);
	}
}

pose_t FriendlyGraph::getPoseEstimate(int pose_id) {
	CHECK_F("bad pose id" && pose_id >= _min_pose_id && pose_id < _max_pose_id);
	pose_t p = _current_guess.block(poseIdx(pose_id), 0, POSE_SIZE, 1);
	return p;
}

void FriendlyGraph::addGPSMeasurement(int pose_id, const transform_t& gps_tf) {
	pose_t gps = toPose(gps_tf, 0); // heading doesn't matter
	_graph.add(new OdomFactor2D(poseIdx(pose_id), -1, _gps_cov_inv, gps));
}

void FriendlyGraph::addOdomMeasurement(int pose2_id, int pose1_id, const transform_t& pose2_tf,
									   const transform_t& pose1_tf) {
	transform_t rel_tf = pose2_tf * pose1_tf.inverse();
	pose_t diff = toPose(rel_tf, 0.0);
	float lin_dist = diff(0);
	// Turning introduces more noise than going in a straight line
	float ang_dist = Constants::WHEEL_BASE * diff(2) * 4;
	float noise_distance_sq = lin_dist * lin_dist + ang_dist * ang_dist;
	_graph.add(new OdomFactor2D(poseIdx(pose2_id), poseIdx(pose1_id),
								_odom_cov_inv / noise_distance_sq, diff));
	pose_t pose1_est = getPoseEstimate(pose1_id);
	transform_t new_pose_tf = rel_tf * toTransform(pose1_est);
	pose_t pose2_est = toPose(new_pose_tf, pose1_est(2));
	_current_guess.block(poseIdx(pose2_id), 0, POSE_SIZE, 1) = pose2_est;
}

void FriendlyGraph::addLandmarkMeasurement(int pose_id, int lm_id, const point_t& bearing,
										   bool overwrite) {
	measurement<2> lm = measurement<2>{bearing(0), bearing(1)};
	if (overwrite && _landmark_factor_ids[lm_id] != -1) {
		_graph.deleteFactor(_landmark_factor_ids[lm_id]);
	}
	int id = _graph.add(
		new LandmarkFactor2D(landmarkIdx(lm_id), poseIdx(pose_id), _sensor_cov_inv, lm));
	_landmark_factor_ids[lm_id] = id;
}

void FriendlyGraph::addLandmarkPrior(int lm_id, point_t location, double xy_std) {
	covariance<2> prior_cov = covariance<2>::Zero();
	prior_cov << xy_std * xy_std, 0, 0, xy_std * xy_std;
	covariance<2> prior_cov_inv = prior_cov.inverse();
	measurement<2> lm = measurement<2>{location(0), location(1)};
	_graph.add(new LandmarkFactor2D(landmarkIdx(lm_id), -1, prior_cov_inv, lm));
	_current_guess.block(landmarkIdx(lm_id), 0, LM_SIZE, 1) = lm;
}

void FriendlyGraph::addPosePrior(int pose_id, const transform_t& pose_tf, covariance<3>& cov) {
	covariance<3> prior_cov_inv = cov.inverse();
	pose_t pose = toPose(pose_tf, 0);
	_graph.add(new OdomFactor2D(poseIdx(pose_id), -1, prior_cov_inv, pose));
	_current_guess.block(poseIdx(pose_id), 0, POSE_SIZE, 1) = pose;
}

// Guarantee: after solve(), _graph.solution() == _current_guess
void FriendlyGraph::solve() {
	trimToMaxNumPoses();
	_graph.solve(_current_guess, 0.99);
	_current_guess = _graph.solution();
}

points_t FriendlyGraph::getLandmarkLocations() {
	points_t lms({});
	for (int i = 0; i < _num_landmarks * LM_SIZE; i += LM_SIZE) {
		point_t lm({0, 0, 1});
		lm.topRows(LM_SIZE) = _current_guess.block(i, 0, LM_SIZE, 1);
		lms.push_back(lm);
	}
	return lms;
}

trajectory_t FriendlyGraph::getSmoothedTrajectory() {
	const values& x = _current_guess;
	trajectory_t tfs({});
	for (int i = poseIdx(_min_pose_id); i < nonincrementingPoseIdx(_max_pose_id);
		 i += POSE_SIZE) {
		if (POSE_SIZE == 3) { // 2D
			tfs.push_back(toTransformRotateFirst(0, 0, x(i + 2)) *
						  toTransformRotateFirst(x(i), x(i + 1), 0));
		} else { // 1D
			tfs.push_back(toTransformRotateFirst(x(i), 0, 0));
		}
	}
	return tfs;
}

int FriendlyGraph::getMaxNumPoses() const {
	return _max_num_poses;
}

} // namespace filters::pose_graph

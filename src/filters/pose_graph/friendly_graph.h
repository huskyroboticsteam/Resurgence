#pragma once

#include "../../navtypes.h"
#include "factors.h"
#include "graph.h"

namespace filters::pose_graph {

class FriendlyGraph {
private:
	int _num_landmarks;
	int _max_pose_id;
	int _min_pose_id;
	int _max_num_poses;
	values _current_guess;
	std::vector<int> _landmark_factor_ids;

	covariance<3> _odom_cov_inv;
	covariance<2> _sensor_cov_inv;
	covariance<3> _gps_cov_inv;

	int nonincrementingPoseIdx(int pose_id);
	int poseIdx(int pose_id);
	int landmarkIdx(int lm_id);
	int numPoses();
	void incrementNumPoses();
	void trimToMaxNumPoses();

public:
	Graph _graph;

	/* num_landmarks: Currently we require you to pre-specify how many landmarks
	 *                will be considered in the pose graph. TODO: Make it possible to
	 *                add more landmarks over time.
	 *
	 * max_num_poses: To prevent the pose graph from growing arbitrarily over time,
	 *                we automatically trim the oldest poses once we get enough newer ones.
	 *                This parameter specifies the maximum number of poses in the graph.
	 *
	 * camera_std:    Standard deviation of landmark distance measurements, in meters
	 *
	 * gps_xy_std:    Standard deviation of GPS (x,y) measurements, in meters
	 *
	 * wheel_noise_rate:
	 *      The variance of odom measurements will depend on how far the robot traveled.
	 *      A WHEEL_NOISE_RATE of 0.05 means that when we rotate a wheel through a distance
	 *      of 1 meter, the standard deviation of the true distance rotated will be 5 cm.
	 */
	FriendlyGraph(int num_landmarks, int max_num_poses, float camera_std, float gps_xy_std,
				  float wheel_noise_rate);

	void addGPSMeasurement(int pose_id, const navtypes::transform_t& gps_tf);
	void addOdomMeasurement(int pose2_id, int pose1_id, const navtypes::transform_t& pose2_tf,
							const navtypes::transform_t& pose1_tf);
	void addLandmarkMeasurement(int pose_id, int lm_id, const navtypes::point_t& bearing,
								bool overwrite = false);
	void addLandmarkPrior(int lm_id, navtypes::point_t location, double xy_std);
	void addPosePrior(int pose_id, const navtypes::transform_t& pose_tf, covariance<3>& cov);

	navtypes::pose_t getPoseEstimate(int pose_id);
	void solve();
	navtypes::points_t getLandmarkLocations();
	/* This method will return the smoothed trajectory (assuming you've already called
	 * `solve()`). Note that the length of this trajectory will be at most `max_num_poses`
	 * (the oldest poses are discarded). */
	navtypes::trajectory_t getSmoothedTrajectory();
	int getMaxNumPoses() const;
};

} // namespace filters::pose_graph

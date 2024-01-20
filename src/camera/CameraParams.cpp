#include "CameraParams.h"

#include "../../src/Constants.h"
#include "../../src/camera/CameraConfig.h"

#include <vector>

#include <opencv2/core.hpp>

namespace cam {

////////////// CONSTRUCTORS //////////////

CameraParams::CameraParams() {}

void CameraParams::init(const cv::Mat& camera_matrix, const cv::Mat& dist_coeff,
						cv::Size image_size) {
	if (camera_matrix.size() != cv::Size(3, 3)) {
		throw std::invalid_argument("Camera matrix must be 3x3");
	}
	int n_coeffs = dist_coeff.rows * dist_coeff.cols;
	if (!(n_coeffs == 4 || n_coeffs == 5 || n_coeffs == 8 || n_coeffs == 12 ||
		  n_coeffs == 14)) {
		throw std::invalid_argument(
			"Number of distortion coefficients must be 4, 5, 8, 12, or 14");
	}
	if (image_size.empty()) {
		throw std::invalid_argument("Image size must not be empty");
	}

	dist_coeff.reshape(1, {n_coeffs, 1}).copyTo(this->_dist_coeff);
	camera_matrix.copyTo(this->_camera_matrix);
	this->_image_size = image_size;
}

CameraParams::CameraParams(const cv::Mat& camera_matrix, const cv::Mat& dist_coeff,
						   cv::Size image_size) {
	init(camera_matrix, dist_coeff, image_size);
}

CameraParams::CameraParams(const CameraParams& other) {
	cv::Mat newCam, newDist;
	other._camera_matrix.copyTo(newCam);
	other._dist_coeff.copyTo(newDist);
	this->_camera_matrix = newCam;
	this->_dist_coeff = newDist;

	this->_image_size = other._image_size;
}

bool CameraParams::empty() const {
	return _camera_matrix.empty() || _dist_coeff.empty() || _image_size.empty();
}

////////////// ACCESSORS /////////////////

cv::Mat CameraParams::getCameraMatrix() const {
	return _camera_matrix;
}

cv::Mat CameraParams::getDistCoeff() const {
	return _dist_coeff;
}

cv::Size CameraParams::getImageSize() const {
	return _image_size;
}

std::vector<double> CameraParams::getIntrinsicList() {
	std::vector<double> intrinsic_list1D;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			double x = _camera_matrix.at<double>(i, j);
			intrinsic_list1D.push_back(x);
		}
	}
	return intrinsic_list1D;
}
////////////// SERIALIZATION //////////////

void CameraParams::readFromFileNode(const cv::FileNode& file_node) {
	cv::Mat cam, dist;
	file_node[KEY_CAMERA_MATRIX] >> cam;
	file_node[KEY_DIST_COEFFS] >> dist;
	int w, h;
	w = (int)file_node[KEY_IMAGE_WIDTH];
	h = (int)file_node[KEY_IMAGE_HEIGHT];
	cv::Size size(w, h);
	// call init to do validation
	init(cam, dist, size);
}

void CameraParams::writeToFileStorage(cv::FileStorage& file_storage) const {
	file_storage << "{";
	file_storage << KEY_IMAGE_WIDTH << _image_size.width;
	file_storage << KEY_IMAGE_HEIGHT << _image_size.height;
	file_storage << KEY_CAMERA_MATRIX << _camera_matrix;
	file_storage << KEY_DIST_COEFFS << _dist_coeff;
	file_storage << "}";
}

void read(const cv::FileNode& node, cam::CameraParams& params,
		  const cam::CameraParams& default_value) {
	if (node.empty()) {
		params = default_value;
	} else {
		params.readFromFileNode(node);
	}
}

void write(cv::FileStorage& fs, __attribute__((unused)) const std::string& name,
		   const cam::CameraParams& params) {
	params.writeToFileStorage(fs);
}

} // namespace cam

#include "base64_img.h"

#include "base64.h"

// code taken from here:
// https://stackoverflow.com/questions/29772271/c-opencv-convert-mat-to-base64-and-vice-versa

namespace base64 {

cv::Mat decodeMat(const std::string& b64) {
	std::string decoded = base64::base64_decode(b64);
	std::vector<uchar> data(decoded.begin(), decoded.end());
	cv::Mat img = cv::imdecode(cv::Mat(data), cv::ImreadModes::IMREAD_COLOR);
	return img;
}

std::string encodeMat(const cv::Mat& mat, const std::string& format) {
	std::vector<uchar> buf;
	cv::imencode(format, mat, buf);
	auto* enc_msg = reinterpret_cast<unsigned char*>(buf.data());
	std::string encoded = base64::base64_encode(enc_msg, buf.size());
	return encoded;
}

} // namespace base64

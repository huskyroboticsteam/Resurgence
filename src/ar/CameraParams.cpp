#include "CameraParams.h"

namespace AR
{

CameraParams::CameraParams(cv::Mat camera_params, cv::Mat dist_coeff)
{
	_camera_params = camera_params;
	_dist_coeff = dist_coeff;
}

cv::Mat CameraParams::getCameraParams() const
{
	return _camera_params;
}

cv::Mat CameraParams::getDistCoeff() const
{
	return _dist_coeff;
}


// -------------------------------------
// Webcam, 640x480, scale in mm
// -------------------------------------
constexpr double _webcam_params[] = {5.5929841506515140e+02, 0., 3.3632998221165434e+02,
									0.,5.5929841506515140e+02, 2.4560280586167062e+02,
									0., 0., 1.};
constexpr double _webcam_dist[] = {2.2319725483552814e-02, -2.3138101190867111e-01,
								  3.6220766734074462e-03, 3.8852893952725500e-03,
								  5.4773015987500950e-01};

const CameraParams WEBCAM_PARAMS(cv::Mat(3, 3, CV_64F, *_webcam_params),
								 cv::Mat(5, 1, CV_64F, *_webcam_dist));
// -------------------------------------


// -------------------------------------
// Laptop webcam, 640x480, scale in mm
// -------------------------------------
constexpr double _laptop_params[] = {5.6534586568739849e+02, 0., 3.5588060437029822e+02,
									0.,5.6534586568739849e+02, 2.9541081641775287e+02,
									0., 0., 1.};
constexpr double _laptop_dist[] = {3.4425270669197583e-01, -4.5627505509852968e+00,
								  1.4729564760154447e-04, -1.2416402052553264e-02,
								  1.5720190712074883e+01};

const CameraParams LAPTOP_PARAMS(cv::Mat(3, 3, CV_64F, *_laptop_params),
								 cv::Mat(5, 1, CV_64F, *_laptop_dist));
// -------------------------------------


// -------------------------------------
// Webcam, 1280x720, scale in m
// -------------------------------------
constexpr double _webcam_720_params[] = {9.3961383346628031e+02, 0., 6.3484000394178668e+02,
										 0., 9.3961383346628031e+02, 3.7840207513366113e+02,
										 0., 0., 1.};
constexpr double _webcam_720_dist[] = {2.1134874734267596e-02, -3.1838385257944879e-01,
									   7.2249055307435119e-03, -9.2194715382554367e-03,
									   6.5454969448093581e-01};

const CameraParams WEBCAM_720_PARAMS(cv::Mat(3, 3, CV_64F, *_webcam_720_params),
									 cv::Mat(5, 1, CV_64F, *_webcam_720_dist));

// -------------------------------------


// -------------------------------------
// Webcam, 1920x1080, scale in m
// -------------------------------------
constexpr double _webcam_1080_params[] = {1.6344311125816394e+03, 0., 9.7545425956890222e+02,
										 0.,1.6344311125816394e+03, 6.4209372992259819e+02,
										 0., 0., 1.};
constexpr double _webcam_1080_dist[] = {-2.1266729721561975e-02, -3.1767566462011504e-01,
									   1.9258722839178691e-02, -3.7013946009933673e-03,
									   7.9803079689451806e-01};

const CameraParams WEBCAM_1080_PARAMS(cv::Mat(3, 3, CV_64F, *_webcam_1080_params),
									  cv::Mat(5, 1, CV_64F, *_webcam_1080_dist));
// -------------------------------------


// -------------------------------------
// Winston's Webcam, 640x480, scale in m
// -------------------------------------
constexpr double _winston_webcam_480_params[] = {6.2703337187188697e+02, 0., 3.2677375666624550e+02,
 												0., 6.2703337187188697e+02, 2.3701509360183852e+02, 
												0., 0., 1.};
constexpr double _winston_webcam_480_dist[] = {8.7938715033125687e-03, 1.6438593467138751e-01,
       										  1.9323488037337649e-04, -3.7075323762211791e-03,
       										  -1.1096023248348679e+00};

const CameraParams WINSTON_WEBCAM_480_PARAMS(cv::Mat(3, 3, CV_64F, *_winston_webcam_480_params),
											 cv::Mat(5, 1, CV_64F, *_winston_webcam_480_dist));
// -------------------------------------

// --------------------------------------
// Evan's new webcam, 640x480, scale in m
// --------------------------------------
constexpr double _evan_new_webcam_480_params[] = {2.8163054058138857e+02, 0., 3.1986287258182375e+02,
												  0., 6.5147998226038069e+01, 2.4050723303632392e+02,
												  0., 0., 1.};
constexpr double _evan_new_webcam_480_dist[] = {-3.6842153449902861e-03, -8.3861698098247113e-05,
												-7.1088419024984209e-03, -4.1784556258417084e-04,
												3.0386330418980679e-07};

const CameraParams EVAN_NEW_WEBCAM_480_PARAMS(cv::Mat(3, 3, CV_64F, *_evan_new_webcam_480_params),
											 cv::Mat(5, 1, CV_64F, *_evan_new_webcam_480_dist));
// -------------------------------------
// --------------------------------------
// Robot top webcamera, 640x480, scale in m
// --------------------------------------
constexpr double _robot_top_webcam_480_params[] = {6.4140478114760754e+02, 0., 3.3489433405024562e+02, 
												0., 6.4140478114760754e+02, 2.4549088452431096e+02, 
												0.,0., 1.};
constexpr double _robot_top_webcam_480_dist[] = {-4.5423809818390071e-01, 3.2556375646404534e-01,
       											6.0432508258992704e-04, -1.1136436683798529e-03,
       											-2.5321322863141504e-01};

const CameraParams ROBOT_TOP_WEBCAM_480_PARAMS(cv::Mat(3, 3, CV_64F, *_robot_top_webcam_480_params),
											 cv::Mat(5, 1, CV_64F, *_robot_top_webcam_480_dist));
// -------------------------------------

CameraParams getCameraParams(Params params)
{
	switch (params)
	{
	case WINSTON_WEBCAM_480:
		return WINSTON_WEBCAM_480_PARAMS;
	case WEBCAM_1080:
		return WEBCAM_1080_PARAMS;
	case WEBCAM_720:
		return WEBCAM_720_PARAMS;
	case LAPTOP:
		return LAPTOP_PARAMS;
	case WEBCAM:
		return WEBCAM_PARAMS;
	case EVAN_NEW_WEBCAM_480:
		return EVAN_NEW_WEBCAM_480_PARAMS;
	case ROBOT_TOP_WEBCAM_480:
		return ROBOT_TOP_WEBCAM_480_PARAMS;
	}
}

} // namespace AR
///////////////////////////////////////////////////////////////////////////////////////
//
// Program to calibrate a camera. Taken from OpenCV sample code and modified by Husky
// Robotics, 2021; original license agreement is below:
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2015, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Copyright (C) 2015, OpenCV Foundation, all rights reserved.
// Copyright (C) 2015, Itseez Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//

#include <cctype>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <time.h>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

const char* usage = " \nexample command line for calibration from a live feed.\n"
					"   calibration  -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe\n"
					" \n"
					" example command line for calibration from a list of stored images:\n"
					"   imagelist_creator image_list.xml *.png\n"
					"   calibration -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe image_list.xml\n"
					" where image_list.xml is the standard OpenCV XML/YAML\n"
					" use imagelist_creator to create the xml or yaml list\n"
					" file consisting of the list of strings, e.g.:\n"
					" \n"
					"<?xml version=\"1.0\"?>\n"
					"<opencv_storage>\n"
					"<images>\n"
					"view000.png\n"
					"view001.png\n"
					"<!-- view002.png -->\n"
					"view003.png\n"
					"view010.png\n"
					"one_extra_view.jpg\n"
					"</images>\n"
					"</opencv_storage>\n";

const char* liveCaptureHelp =
	"When the live video from camera is used as input, the following hot-keys may be used:\n"
	"  <ESC>, 'q' - quit the program\n"
	"  'g' - start capturing images\n"
	"  'u' - switch undistortion on/off\n";

static void help(char** argv) {
	// clang-format off
    printf( "This is a camera calibration sample.\n"
        "Usage: %s\n"
        "     -w=<board_width>         # the number of inner corners per one of board dimension\n"
        "     -h=<board_height>        # the number of inner corners per another board dimension\n"
        "     [-pt=<pattern>]          # the type of pattern: chessboard or circles' grid\n"
        "     [-n=<number_of_frames>]  # the number of frames to use for calibration\n"
        "                              # (if not specified, it will be set to the number\n"
        "                              #  of board views actually available)\n"
        "     [-d=<delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
        "                              # (used only for video capturing)\n"
        "     [-s=<squareSize>]       # square size in some user-defined units (1 by default)\n"
        "     [-o=<out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
        "     [-op]                    # write detected feature points\n"
        "     [-oe]                    # write extrinsic parameters\n"
        "     [-oo]                    # write refined 3D object points\n"
        "     [-zt]                    # assume zero tangential distortion\n"
        "     [-a=<aspectRatio>]      # fix aspect ratio (fx/fy)\n"
        "     [-p]                     # fix the principal point at the center\n"
        "     [-v]                     # flip the captured images around the horizontal axis\n"
        "     [-V]                     # use a video file, and not an image list, uses\n"
        "                              # [input_data] string for the video file name\n"
        "     [-su]                    # show undistorted images after calibration\n"
        "     [-ws=<number_of_pixel>]  # Half of search window for cornerSubPix (11 by default)\n"
        "     [-dt=<distance>]         # actual distance between top-left and top-right corners of\n"
        "                              # the calibration grid. If this parameter is specified, a more\n"
        "                              # accurate calibration method will be used which may be better\n"
        "                              # with inaccurate, roughly planar target.\n"
        "     [input_data]             # input data, one of the following:\n"
        "                              #  - text file with a list of the images of the board\n"
        "                              #    the text file can be generated with imagelist_creator\n"
        "                              #  - name of video file with a video of the board\n"
        "                              # if input_data not specified, a live view from the camera is used\n"
			"\n" ,argv[0]);
	// clang-format on
	printf("\n%s", usage);
	printf("\n%s", liveCaptureHelp);
}

enum {
	DETECTION = 0,
	CAPTURING = 1,
	CALIBRATED = 2
};
enum Pattern {
	CHESSBOARD,
	CIRCLES_GRID,
	ASYMMETRIC_CIRCLES_GRID
};

static double
computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
						  const std::vector<std::vector<cv::Point2f>>& imagePoints,
						  const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
						  const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
						  std::vector<float>& perViewErrors) {
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < static_cast<int>(objectPoints.size()); i++) {
		projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs,
					  imagePoints2);
		err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
		int n = static_cast<int>(objectPoints[i].size());
		perViewErrors[i] = static_cast<float>(std::sqrt(err * err / n));
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcChessboardCorners(cv::Size boardSize, float squareSize,
								  std::vector<cv::Point3f>& corners,
								  Pattern patternType = CHESSBOARD) {
	corners.resize(0);

	switch (patternType) {
		case CHESSBOARD:
		case CIRCLES_GRID:
			for (int i = 0; i < boardSize.height; i++)
				for (int j = 0; j < boardSize.width; j++)
					corners.push_back(
						cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
			break;

		case ASYMMETRIC_CIRCLES_GRID:
			for (int i = 0; i < boardSize.height; i++)
				for (int j = 0; j < boardSize.width; j++)
					corners.push_back(cv::Point3f(float((2 * j + i % 2) * squareSize),
												  float(i * squareSize), 0));
			break;

		default:
			CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
	}
}

static bool runCalibration(std::vector<std::vector<cv::Point2f>> imagePoints,
						   cv::Size imageSize, cv::Size boardSize, Pattern patternType,
						   float squareSize, float aspectRatio, float grid_width,
						   bool release_object, int flags, cv::Mat& cameraMatrix,
						   cv::Mat& distCoeffs, std::vector<cv::Mat>& rvecs,
						   std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs,
						   std::vector<cv::Point3f>& newObjPoints, double& totalAvgErr) {
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	if (flags & cv::CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = aspectRatio;

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f>> objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
	objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
	newObjPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms;
	int iFixedPoint = -1;
	if (release_object)
		iFixedPoint = boardSize.width - 1;
	rms = cv::calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
								cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
								flags | cv::CALIB_FIX_K3 | cv::CALIB_USE_LU);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	if (release_object) {
		std::cout << "New board corners: " << std::endl;
		std::cout << newObjPoints[0] << std::endl;
		std::cout << newObjPoints[boardSize.width - 1] << std::endl;
		std::cout << newObjPoints[boardSize.width * (boardSize.height - 1)] << std::endl;
		std::cout << newObjPoints.back() << std::endl;
	}

	objectPoints.clear();
	objectPoints.resize(imagePoints.size(), newObjPoints);
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs,
											cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

static void
saveCameraParams(const std::string& filename, cv::Size imageSize, cv::Size boardSize,
				 float squareSize, float aspectRatio, int flags, const cv::Mat& cameraMatrix,
				 const cv::Mat& distCoeffs, const std::vector<cv::Mat>& rvecs,
				 const std::vector<cv::Mat>& tvecs, const std::vector<float>& reprojErrs,
				 const std::vector<std::vector<cv::Point2f>>& imagePoints,
				 const std::vector<cv::Point3f>& newObjPoints, double totalAvgErr) {
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	fs.writeComment("Name and description for the camera.\nYou should replace these.");
	fs << "name"
	   << "TODO: Replace this"
	   << "description"
	   << "TODO: Replace this";
	fs.writeComment("Additionally, you should choose one of the below\n"
					"and replace it with the proper value.");
	fs.writeComment("Camera ID: given ID N, will attempt to use /dev/videoN.");
	fs << "camera_id" << 0;
	fs.writeComment("Filename: will attempt to open the video device/stream\n"
					"at the given path.");
	fs << "filename"
	   << "/path/to/camera/device";
	
	fs.writeComment("Video format: typically 'image/jpeg' or 'video/x-raw'.");
	fs << "format" << "image/jpeg";
	fs.writeComment("Framerate in frames per second.");
	fs << "framerate" << 30;
	fs.writeComment("Image dimensions (must match calibration resolution).");
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;

	time_t tt;
	time(&tt);
	struct tm* t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs.writeComment("Information about the calibration process, such as the\n"
					"reprojection error and board size.");
	fs << "calib_info"
	   << "{";
	fs << "calibration_time" << buf;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << static_cast<int>(std::max(rvecs.size(), reprojErrs.size()));

	if (flags & cv::CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s",
				flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
				flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
				flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
				flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		// cvWriteComment( *fs, buf, 0 );
	}

	fs << "flags" << flags;
	fs << "}";

	fs.writeComment("The actual intrinsic parameters of the camera.");
	fs << "intrinsic_params"
	   << "{";
	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;

	fs << "}";

	if (!rvecs.empty() && !tvecs.empty()) {
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		cv::Mat bigmat(static_cast<int>(rvecs.size()), 6, rvecs[0].type());
		for (int i = 0; i < static_cast<int>(rvecs.size()); i++) {
			cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
			cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		// cvWriteComment( *fs, "a set of 6-tuples (rotation std::vector + translation
		// std::vector) for each view", 0 );
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty()) {
		cv::Mat imagePtMat(static_cast<int>(imagePoints.size()),
						   static_cast<int>(imagePoints[0].size()), CV_32FC2);
		for (int i = 0; i < static_cast<int>(imagePoints.size()); i++) {
			cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			cv::Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}

	if (!newObjPoints.empty()) {
		fs << "grid_points" << newObjPoints;
	}
}

static bool readStringList(const std::string& filename, std::vector<std::string>& l) {
	l.resize(0);
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	size_t dir_pos = filename.rfind('/');
	if (dir_pos == std::string::npos)
		dir_pos = filename.rfind('\\');
	cv::FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != cv::FileNode::SEQ)
		return false;
	cv::FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it) {
		std::string fname = (std::string)*it;
		if (dir_pos != std::string::npos) {
			std::string fpath =
				cv::samples::findFile(filename.substr(0, dir_pos + 1) + fname, false);
			if (fpath.empty()) {
				fpath = cv::samples::findFile(fname);
			}
			fname = fpath;
		} else {
			fname = cv::samples::findFile(fname);
		}
		l.push_back(fname);
	}
	return true;
}

static bool runAndSave(const std::string& outputFilename,
					   const std::vector<std::vector<cv::Point2f>>& imagePoints,
					   cv::Size imageSize, cv::Size boardSize, Pattern patternType,
					   float squareSize, float grid_width, bool release_object,
					   float aspectRatio, int flags, cv::Mat& cameraMatrix,
					   cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints,
					   bool writeGrid) {
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;
	std::vector<cv::Point3f> newObjPoints;

	bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
							 aspectRatio, grid_width, release_object, flags, cameraMatrix,
							 distCoeffs, rvecs, tvecs, reprojErrs, newObjPoints, totalAvgErr);
	printf("%s. avg reprojection error = %.7f\n",
		   ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

	if (ok)
		saveCameraParams(outputFilename, imageSize, boardSize, squareSize, aspectRatio, flags,
						 cameraMatrix, distCoeffs,
						 writeExtrinsics ? rvecs : std::vector<cv::Mat>(),
						 writeExtrinsics ? tvecs : std::vector<cv::Mat>(),
						 writeExtrinsics ? reprojErrs : std::vector<float>(),
						 writePoints ? imagePoints : std::vector<std::vector<cv::Point2f>>(),
						 writeGrid ? newObjPoints : std::vector<cv::Point3f>(), totalAvgErr);
	return ok;
}

int main(int argc, char** argv) {
	cv::Size boardSize, imageSize;
	float squareSize, aspectRatio = 1;
	cv::Mat cameraMatrix, distCoeffs;
	std::string outputFilename;
	std::string inputFilename = "";

	int i, nframes;
	bool writeExtrinsics, writePoints;
	bool undistortImage = false;
	int flags = 0;
	cv::VideoCapture capture;
	bool flipVertical;
	bool showUndistorted;
	bool videofile;
	int delay;
	clock_t prevTimestamp = 0;
	int mode = DETECTION;
	int cameraId = 0;
	std::vector<std::vector<cv::Point2f>> imagePoints;
	std::vector<std::string> imageList;
	Pattern pattern = CHESSBOARD;

	cv::CommandLineParser parser(
		argc, argv,
		"{help ||}{w||}{h||}{pt|chessboard|}{n|10|}{d|1000|}{s|1|}{o|out_camera_data.yml|}"
		"{op||}{oe||}{zt||}{a||}{p||}{v||}{V||}{su||}"
		"{oo||}{ws|11|}{dt||}"
		"{@input_data|0|}");
	if (parser.has("help")) {
		help(argv);
		return 0;
	}
	boardSize.width = parser.get<int>("w");
	boardSize.height = parser.get<int>("h");
	if (parser.has("pt")) {
		std::string val = parser.get<std::string>("pt");
		if (val == "circles")
			pattern = CIRCLES_GRID;
		else if (val == "acircles")
			pattern = ASYMMETRIC_CIRCLES_GRID;
		else if (val == "chessboard")
			pattern = CHESSBOARD;
		else
			return fprintf(stderr, "Invalid pattern type: must be chessboard or circles\n"),
				   -1;
	}
	squareSize = parser.get<float>("s");
	nframes = parser.get<int>("n");
	delay = parser.get<int>("d");
	writePoints = parser.has("op");
	writeExtrinsics = parser.has("oe");
	bool writeGrid = parser.has("oo");
	if (parser.has("a")) {
		flags |= cv::CALIB_FIX_ASPECT_RATIO;
		aspectRatio = parser.get<float>("a");
	}
	if (parser.has("zt"))
		flags |= cv::CALIB_ZERO_TANGENT_DIST;
	if (parser.has("p"))
		flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
	flipVertical = parser.has("v");
	videofile = parser.has("V");
	if (parser.has("o"))
		outputFilename = parser.get<std::string>("o");
	showUndistorted = parser.has("su");
	if (isdigit(parser.get<std::string>("@input_data")[0]))
		cameraId = parser.get<int>("@input_data");
	else
		inputFilename = parser.get<std::string>("@input_data");
	int winSize = parser.get<int>("ws");
	float grid_width = squareSize * (boardSize.width - 1);
	bool release_object = false;
	if (parser.has("dt")) {
		grid_width = parser.get<float>("dt");
		release_object = true;
	}
	if (!parser.check()) {
		help(argv);
		parser.printErrors();
		return -1;
	}
	if (squareSize <= 0)
		return fprintf(stderr, "Invalid board square width\n"), -1;
	if (nframes <= 3)
		return printf("Invalid number of images\n"), -1;
	if (aspectRatio <= 0)
		return printf("Invalid aspect ratio\n"), -1;
	if (delay <= 0)
		return printf("Invalid delay\n"), -1;
	if (boardSize.width <= 0)
		return fprintf(stderr, "Invalid board width\n"), -1;
	if (boardSize.height <= 0)
		return fprintf(stderr, "Invalid board height\n"), -1;

	if (!inputFilename.empty()) {
		if (!videofile && readStringList(cv::samples::findFile(inputFilename), imageList))
			mode = CAPTURING;
		else
			capture.open(cv::samples::findFileOrKeep(inputFilename));
	} else
		capture.open(cameraId);

	if (!capture.isOpened() && imageList.empty())
		return fprintf(stderr, "Could not initialize video (%d) capture\n", cameraId), -2;

	if (!imageList.empty())
		nframes = static_cast<int>(imageList.size());

	if (capture.isOpened())
		printf("%s", liveCaptureHelp);

	imageSize = {640, 480}; // TODO change this

	capture.set(cv::CAP_PROP_FRAME_WIDTH, imageSize.width);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, imageSize.height);

	cv::namedWindow("Image View", 1);

	for (i = 0;; i++) {
		cv::Mat view, viewGray;
		bool blink = false;

		if (capture.isOpened()) {
			cv::Mat view0;
			capture >> view0;
			view0.copyTo(view);
		} else if (i < static_cast<int>(imageList.size()))
			view = cv::imread(imageList[i], 1);

		if (view.empty()) {
			if (imagePoints.size() > 0)
				runAndSave(outputFilename, imagePoints, imageSize, boardSize, pattern,
						   squareSize, grid_width, release_object, aspectRatio, flags,
						   cameraMatrix, distCoeffs, writeExtrinsics, writePoints, writeGrid);
			break;
		}

		imageSize = view.size();

		if (flipVertical)
			flip(view, view, 0);

		std::vector<cv::Point2f> pointbuf;
		cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);

		bool found;
		switch (pattern) {
			case CHESSBOARD:
				found = findChessboardCorners(view, boardSize, pointbuf,
											  cv::CALIB_CB_ADAPTIVE_THRESH |
												  cv::CALIB_CB_FAST_CHECK |
												  cv::CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				found = findCirclesGrid(view, boardSize, pointbuf);
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				found =
					findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID);
				break;
			default:
				return fprintf(stderr, "Unknown pattern type\n"), -1;
		}

		// improve the found corners' coordinate accuracy
		if (pattern == CHESSBOARD && found)
			cornerSubPix(
				viewGray, pointbuf, cv::Size(winSize, winSize), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));

		if (mode == CAPTURING && found &&
			(!capture.isOpened() || clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC)) {
			imagePoints.push_back(pointbuf);
			prevTimestamp = clock();
			blink = capture.isOpened();
		}

		if (found)
			drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);

		std::string msg = mode == CAPTURING	   ? "100/100"
						  : mode == CALIBRATED ? "Calibrated"
											   : "Press 'g' to start";
		int baseLine = 0;
		cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
		cv::Point textOrigin(view.cols - 2 * textSize.width - 10,
							 view.rows - 2 * baseLine - 10);

		if (mode == CAPTURING) {
			if (undistortImage)
				msg =
					cv::format("%d/%d Undist", static_cast<int>(imagePoints.size()), nframes);
			else
				msg = cv::format("%d/%d", static_cast<int>(imagePoints.size()), nframes);
		}

		putText(view, msg, textOrigin, 1, 1,
				mode != CALIBRATED ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

		if (blink)
			bitwise_not(view, view);

		if (mode == CALIBRATED && undistortImage) {
			cv::Mat temp = view.clone();
			undistort(temp, view, cameraMatrix, distCoeffs);
		}

		imshow("Image View", view);
		char key = static_cast<char>(cv::waitKey(capture.isOpened() ? 50 : 500));

		if (key == 27)
			break;

		if (key == 'u' && mode == CALIBRATED)
			undistortImage = !undistortImage;

		if (capture.isOpened() && key == 'g') {
			mode = CAPTURING;
			imagePoints.clear();
		}

		if (mode == CAPTURING && imagePoints.size() >= (unsigned)nframes) {
			if (runAndSave(outputFilename, imagePoints, imageSize, boardSize, pattern,
						   squareSize, grid_width, release_object, aspectRatio, flags,
						   cameraMatrix, distCoeffs, writeExtrinsics, writePoints, writeGrid))
				mode = CALIBRATED;
			else
				mode = DETECTION;
			if (!capture.isOpened())
				break;
		}
	}

	if (!capture.isOpened() && showUndistorted) {
		cv::Mat view, rview, map1, map2;
		initUndistortRectifyMap(
			cameraMatrix, distCoeffs, cv::Mat(),
			getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
			imageSize, CV_16SC2, map1, map2);

		for (i = 0; i < static_cast<int>(imageList.size()); i++) {
			view = cv::imread(imageList[i], 1);
			if (view.empty())
				continue;
			// undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
			remap(view, rview, map1, map2, cv::INTER_LINEAR);
			imshow("Image View", rview);
			char c = static_cast<char>(cv::waitKey());
			if (c == 27 || c == 'q' || c == 'Q')
				break;
		}
	}

	return 0;
}

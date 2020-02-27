#pragma once

#include "CameraParams.h"
#include "Tag.h"

#include <opencv2/core.hpp>

#ifdef WITH_GPU
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

#include <vector>

namespace AR
{
class Detector
{
	private:
		CameraParams cam;
		#ifdef WITH_GPU
			cv::Ptr<cv::cuda::CannyEdgeDetector> detector;
			cv::Ptr<cv::cuda::createMorphologyFilter> filter;
		#endif

	public:
		Detector(CameraParams params);
		
		/**
		   @brief Find Tags in the given image.
		   
		   This version of the function is mostly used for interactive testing programs that
		   need to have a copy of the images at various processing stages.

		   Most parameters are optional and are mainly intended for testing. For general use it
		   should be sufficient to simply pass in a Mat containing the image.

		   @param input The image in which to detect AR tags.
		   @param grayscale A reference to a Mat where the grayscaled and blurred image should
		   be stored.
		   @param edges A reference to a Mat where the edges from Canny edge detection should
		   be stored.
		   @param canny_thresh_1 Optional, the first threshold parameter for Canny edge
		   detection.
		   @param canny_thresh_2 Optional, the second threshold parameter for Canny edge
		   detection.
		   @param blur_size Optional, the size of the blur used. MUST BE AN ODD NUMBER.
		*/
		std::vector<Tag> findTags(cv::Mat input, cv::Mat &grayscale, cv::Mat &edges,
								  std::vector<std::vector<cv::Point2f> > &quad_corners,
								  int canny_thresh_1 = 50, int canny_thresh_2 = 120,
								  int blur_size = 5);

		/**
		   @brief Find Tags in the given image.

		   Most parameters are optional and are mainly intended for testing. For general use it
		   should be sufficient to simply pass in a Mat containing the image.

		   @param input The image in which to detect AR tags.
		   @param canny_thresh_1 Optional, the first threshold parameter for Canny edge
		   detection.
		   @param canny_thresh_2 Optional, the second threshold parameter for Canny edge
		   detection.
		   @param blur_size Optional, the size of the blur used. MUST BE AN ODD NUMBER.
		*/
		std::vector<Tag> findTags(cv::Mat input, int canny_thresh_1 = 50,
								  int canny_thresh_2 = 120, int blur_size = 5);
};
} // namespace AR

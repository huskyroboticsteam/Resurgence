#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../camera/Camera.h"

const std::string WINDOW_NAME = "Test";

int main() {
    cv::VideoCapture cap(0);
    // cam::Camera c;
    // c.open(40);

    // cap.open(0);

    cv::namedWindow(WINDOW_NAME);

    cv::Mat frame;
    // uint32_t frame_num;

    while (true) {
        if (!cap.grab()) { continue; }

        cap.retrieve(frame);

        if (frame.empty()) { continue; }

        // c.next(frame, frame_num);
        cv::imshow(WINDOW_NAME, frame);

        cv::waitKey(0);
    }
}
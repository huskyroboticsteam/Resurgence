#include "Camera.h"

#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#include <opencv2/highgui.hpp>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

// TODO: add more comments to this program

const string WINDOW_TITLE_BASE = "Window ";

string win(int num) {
	return WINDOW_TITLE_BASE + std::to_string(num);
}

const string KEYS = "{h help || Show this help message.}"
					"{n | 3 | The number of windows to open.}"
					"{cam c | 0 | The camera ID to use.}";
const string ABOUT = "Program to open a camera and test concurrent access.";

int main(int argc, char** argv) {
	cv::CommandLineParser parser(argc, argv, KEYS);
	if (!parser.check()) {
		parser.printErrors();
	}
	parser.about(ABOUT);

	if (parser.has("h")) {
		parser.printMessage();
		return EXIT_SUCCESS;
	}

	int camera_id = parser.get<int>("c");
	cout << "Opening camera..." << endl;
	cam::Camera camera(camera_id, "Test camera", "Test camera");
	if (!camera.isOpen()) {
		cerr << "ERROR! Unable to open camera" << endl;
		return EXIT_FAILURE;
	}

	vector<cam::Camera> camera_instances;
	vector<uint32_t> frame_nums;

	int n = parser.get<int>("n");
	cv::namedWindow(win(0));
	uint32_t original_frame_num = 0;
	for (int i = 0; i < n; i++) {
		cv::namedWindow(win(i + 1));
		camera_instances.push_back(camera);
		frame_nums.push_back(0);
	}

	bool running = true;
	while (running) {
		for (int i = 0; i < n; i++) {
			cv::Mat frame;
			if (camera_instances[i].hasNext(frame_nums[i])) {
				camera_instances[i].next(frame, frame_nums[i]);
				if (frame.empty()) {
					cerr << "Empty frame from camera " << i << endl;
				} else {
					cv::imshow(win(i + 1), frame);
				}
			}
		}
		cv::Mat original_frame;
		if (camera.hasNext(original_frame_num)) {
			camera.next(original_frame, original_frame_num);
			if (original_frame.empty()) {
				cerr << "Empty frame from original camera" << endl;
			} else {
				cv::imshow(win(0), original_frame);
			}
		}
		switch (cv::waitKey(1)) {
			case 'q':
				running = false;
				break;
		}
	}

	return EXIT_SUCCESS;
}

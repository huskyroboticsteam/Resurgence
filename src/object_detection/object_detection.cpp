#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>

#include "../world_interface/world_interface.h"
#include "../camera/Camera.h"
#include "../Constants.h"

// Function to preprocess a given image into the format required by the owl-vit model
torch::Tensor preprocess(const cv::Mat& image) {
	const std::vector<float> mean = {0.5, 0.5, 0.5};
	const std::vector<float> std = {0.5, 0.5, 0.5};
	cv::Mat resized, rgb_image;
	cv::resize(image, resized, cv::Size(768, 768));
	cv::cvtColor(resized, rgb_image, cv::COLOR_BGR2RGB);

	torch::Tensor img_tensor =
		torch::from_blob(rgb_image.data, {rgb_image.rows, rgb_image.cols, 3}, torch::kByte);

	img_tensor = img_tensor.permute({2, 0, 1}).to(torch::kFloat32) / 255.0;

	for (int i = 0; i < 3; ++i) {
		img_tensor[i] = (img_tensor[i] - mean[i]) / std[i];
	}

	return img_tensor.unsqueeze(0);
}

// Function to run a given image through an object detection model
// Retruns the logits and predicted bounding boxes for detected objects
std::vector<torch::Tensor> run_owlvit_model(torch::jit::script::Module& model,
											const cv::Mat& image) {
	torch::NoGradGuard no_grad;

	// auto processing_device = torch::kCUDA;
	auto processing_device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;

	// Preprocess image and move it to the processing device (CPU OR CUDA GPU)
	torch::Tensor pixel_values = preprocess(image).to(processing_device);

	// Hardcoded pre-tokenized inputs - ["no object", "orange mallet or hammer", "water
	// bottle"]
	torch::Tensor input_ids =
		torch::tensor({{49406, 871, 14115, 49407, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
					   {49406, 4287, 1662, 1094, 541, 9401, 49407, 0, 0, 0, 0, 0, 0, 0, 0, 0},
					   {49406, 1573, 5392, 49407, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
					  torch::kInt64)
			.to(processing_device);

	torch::Tensor attention_mask =
		torch::tensor({{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
					   {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
					   {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
					  torch::kInt64)
			.to(processing_device);

	// Forward pass
	std::vector<torch::jit::IValue> inputs;
	inputs.push_back(input_ids);
	inputs.push_back(pixel_values);
	inputs.push_back(attention_mask);

	auto outputs = model.forward(inputs);

	auto output_tuple = outputs.toTuple()->elements();
	auto logits = output_tuple[0].toTensor();
	auto pred_boxes = output_tuple[1].toTensor();

	return std::vector<at::Tensor>{logits, pred_boxes};
}

// Function to draw a bounding box at the given position with a label and confidence.
void draw_bounding_box(cv::Mat& image, int x1, int y1, int x2, int y2,
					   const std::string& label, float confidence) {
	// Draw rectangle
	cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

	// Prepare label text
	std::string text = label + " " + std::to_string(confidence * 100).substr(0, 4) + "%";

	// Background rectangle for text
	int baseLine;
	cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	cv::rectangle(image, cv::Point(x1, y1 - textSize.height - baseLine),
				  cv::Point(x1 + textSize.width, y1), cv::Scalar(0, 255, 0), cv::FILLED);

	// Put label text
	cv::putText(image, text, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
				cv::Scalar(0, 0, 0), 1);
}

// Function to draw a bounding box around the object detected with the highest confidence
// value.
void draw_detected_objects(cv::Mat& image, torch::Tensor logits, torch::Tensor boxes,
						   const std::vector<std::string>& class_names,
						   float confidence_threshold = 0.5) {
	int img_width = image.cols;
	int img_height = image.rows;

	// Convert logits to probabilities
	torch::Tensor scores = torch::softmax(logits.squeeze(0), 1);

	// Variables to track the maximum confidence box
	float max_confidence = 0.0;
	int max_class_idx = -1;
	int max_box_idx = -1;

	for (int i = 0; i < scores.size(0); i++) {
		auto max_result = scores[i].max(0);
		float confidence = std::get<0>(max_result).item<float>();
		int class_idx = std::get<1>(max_result).item<int>();

		// Check for the highest confidence and non-background class (class_idx != 0 because 0
		// is "no object")
		if (class_idx != 0 && confidence > max_confidence &&
			confidence > confidence_threshold) {
			max_confidence = confidence;
			max_class_idx = class_idx;
			max_box_idx = i;
		}
	}

	// If a valid box is found, draw it
	if (max_box_idx != -1) {
		auto box = boxes[0][max_box_idx];
		float x_center = box[0].item<float>();
		float y_center = box[1].item<float>();
		float width = box[2].item<float>();
		float height = box[3].item<float>();

		// Convert normalized coordinates to pixel coordinates
		int x1 = static_cast<int>((x_center - width / 2) * img_width);
		int y1 = static_cast<int>((y_center - height / 2) * img_height);
		int x2 = static_cast<int>((x_center + width / 2) * img_width);
		int y2 = static_cast<int>((y_center + height / 2) * img_height);

		// Ensure coordinates are within image boundaries
		x1 = std::max(0, std::min(x1, img_width - 1));
		y1 = std::max(0, std::min(y1, img_height - 1));
		x2 = std::max(0, std::min(x2, img_width - 1));
		y2 = std::max(0, std::min(y2, img_height - 1));

		// Draw the bounding box with label
		std::string label = class_names[max_class_idx];
		draw_bounding_box(image, x1, y1, x2, y2, label, max_confidence);

		// Print the detection for debugging
		std::cout << "Highest Confidence Detected: " << label << " (" << max_confidence * 100
				  << "%) at [" << x1 << ", " << y1 << ", " << x2 << ", " << y2 << "]"
				  << std::endl;
	} else {
		std::cout << "No high-confidence object detected." << std::endl;
	}
}

int main(int argc, char* argv[]) {
	// if (argc < 2) {
	// 	std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
	// 	return -1;
	// }
	// char* image_path = argv[1];

	// Load the model
	torch::jit::script::Module model;
	std::string model_file = "owlvit-cpp.pt";
	try {
		// Deserialize the ScriptModule from a file using torch::jit::load().
		model = torch::jit::load(model_file);
	} catch (const c10::Error& e) {
		std::cerr << "error loading the model " << model_file << "\n";
		std::cerr << e.what() << "\n";
		return -1;
	}
	std::cout << "Model loaded successfully\n";

	// cv::Mat img = cv::imread(image_path);
	robot::openCamera(Constants::MAST_CAMERA_ID);
	while(1) {
		auto camDP = robot::readCamera(Constants::MAST_CAMERA_ID);
		auto data = camDP.getData();
		uint32_t& new_frame_num = data.second;
		cv::Mat frame = data.first;

		auto outputs = run_owlvit_model(model, frame);

		auto logits = outputs[0];
		auto boxes = outputs[1];

		std::vector<std::string> class_names = {"no object", "orange mallet or hammer",
												"water bottle"};

		draw_detected_objects(frame, logits, boxes, class_names, 0.5);

		cv::imshow("Detected Objects", frame);
	}
	

	return 0;
}

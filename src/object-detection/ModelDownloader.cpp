#include "ModelDownloader.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace ObjDet {

bool downloadModel(const std::string& destination) {
    std::cout << "\nDownloading OWL-ViT model..." << std::endl;
    std::cout << "This may take a few minutes depending on your internet connection." << std::endl;
    
    // Create directory if it doesn't exist
    std::filesystem::path dest_path(destination);
    std::filesystem::create_directories(dest_path.parent_path());
    
    // Model download URL
    // You can set OWLVIT_MODEL_URL environment variable to override this
    const char* env_url = std::getenv("OWLVIT_MODEL_URL");
    std::string MODEL_URL;
    
    if (env_url) {
        MODEL_URL = env_url;
    } else {
        // Default: Download from Hugging Face repository
        MODEL_URL = "https://huggingface.co/thomas0829/OWL-ViT/resolve/main/owlvit-cpp.pt";
    }
    
    std::cout << "Downloading from: " << MODEL_URL << std::endl;
    
    // Try wget first, then curl
    std::string download_cmd;
    if (system("which wget > /dev/null 2>&1") == 0) {
        download_cmd = "wget -q --show-progress -O \"" + destination + "\" \"" + MODEL_URL + "\"";
    } else if (system("which curl > /dev/null 2>&1") == 0) {
        download_cmd = "curl -L -o \"" + destination + "\" \"" + MODEL_URL + "\"";
    } else {
        std::cerr << "Error: Neither wget nor curl is installed." << std::endl;
        std::cerr << "Please install wget or curl to download the model." << std::endl;
        std::cerr << "  Ubuntu/Debian: sudo apt install wget" << std::endl;
        std::cerr << "Or manually download from: " << MODEL_URL << std::endl;
        return false;
    }
    
    // Execute download
    int result = system(download_cmd.c_str());
    
    if (result == 0 && std::filesystem::exists(destination)) {
        auto file_size = std::filesystem::file_size(destination);
        std::cout << "Model downloaded successfully!" << std::endl;
        std::cout << "Location: " << destination << std::endl;
        std::cout << "Size: " << (file_size / 1024 / 1024) << " MB" << std::endl;
        return true;
    } else {
        std::cerr << "Failed to download model." << std::endl;
        std::cerr << "\nManual download instructions:" << std::endl;
        std::cerr << "1. Download the model from: " << MODEL_URL << std::endl;
        std::cerr << "2. Save it to: " << destination << std::endl;
        std::cerr << "3. Or set OWLVIT_MODEL_PATH environment variable" << std::endl;
        return false;
    }
}

std::string findOrDownloadModel() {
    // 1. Check environment variable
    const char* env_path = std::getenv("OWLVIT_MODEL_PATH");
    if (env_path && std::filesystem::exists(env_path)) {
        return env_path;
    }
    
    // 2. Check current directory (for backward compatibility)
    if (std::filesystem::exists("owlvit-cpp.pt")) {
        return "owlvit-cpp.pt";
    }
    
    // 3. Check in ../src/object-detection/ (when running from build/)
    if (std::filesystem::exists("../src/object-detection/owlvit-cpp.pt")) {
        return "../src/object-detection/owlvit-cpp.pt";
    }
    
    // 4. Check in src/object-detection/ (when running from project root)
    if (std::filesystem::exists("src/object-detection/owlvit-cpp.pt")) {
        return "src/object-detection/owlvit-cpp.pt";
    }
    
    // Model not found - attempt to download
    std::cout << "OWL-ViT model not found. Attempting automatic download..." << std::endl;
    
    // Determine best download location
    std::string download_path;
    if (std::filesystem::exists("../src/object-detection")) {
        download_path = "../src/object-detection/owlvit-cpp.pt";
    } else if (std::filesystem::exists("src/object-detection")) {
        download_path = "src/object-detection/owlvit-cpp.pt";
    } else {
        download_path = "owlvit-cpp.pt";
    }
    
    if (downloadModel(download_path)) {
        return download_path;
    }
    
    throw std::runtime_error("Could not find or download owlvit-cpp.pt model file. "
                           "Please set OWLVIT_MODEL_PATH environment variable or manually download the model.");
}

} // namespace ObjDet

#pragma once

#include <string>

namespace ObjDet {

/**
 * @brief Find or download the OWL-ViT model file.
 * 
 * This function checks for the model in multiple locations:
 * 1. Environment variable OWLVIT_MODEL_PATH
 * 2. Current directory
 * 3. ../src/object-detection/ (from build/)
 * 4. src/object-detection/ (from project root)
 * 
 * If the model is not found, it will automatically download from Hugging Face.
 * 
 * @return Path to the model file
 * @throws std::runtime_error if model cannot be found or downloaded
 */
std::string findOrDownloadModel();

/**
 * @brief Download OWL-ViT model from Hugging Face to destination.
 * 
 * Downloads the pre-converted TorchScript model using wget or curl.
 * The download URL can be overridden with OWLVIT_MODEL_URL environment variable.
 * 
 * @param destination Path where the model should be saved
 * @return true if download successful, false otherwise
 */
bool downloadModel(const std::string& destination);

} // namespace ObjDet

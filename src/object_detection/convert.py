#Script to generate the owlvit - cpp model for C++

from transformers import OwlViTProcessor, OwlViTForObjectDetection, OwlViTConfig
from PIL import Image
import torch

model_name = "google/owlvit-base-patch32"
processor = OwlViTProcessor.from_pretrained(model_name)

#Move to GPU if available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
detector = OwlViTForObjectDetection.from_pretrained(model_name, config=OwlViTConfig(return_dict=False)).to(device)

classes = ["no object", "orange mallet or hammer", "water bottle"]
#Make sure to download a mallet.jpg image and place it in the same directory as this script.
example = processor(text=classes, images=Image.open("mallet.jpg"), return_tensors="pt")
example = {k: v.to(device) if isinstance(v, torch.Tensor) else v for k, v in example.items()}

traced_script_module = torch.jit.trace(detector, (example["input_ids"], example["pixel_values"], example["attention_mask"]))

file_destination = "../../build/owlvit-cpp.pt"
traced_script_module.save(file_destination)
print(f"Saved model in {file_destination}")

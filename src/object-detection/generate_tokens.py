#!/usr/bin/env python3
"""
Script to generate hard coded input tokens (Input IDs and Attention Masks) to prompt owl-vit.

Usage:
    python generate_tokens.py                    # Use default descriptions below
    python generate_tokens.py "custom item"      # Custom descriptions  
    python generate_tokens.py -n "item1" "item2" # Add 'no object' prefix
"""

from transformers import OwlViTProcessor, OwlViTForObjectDetection, OwlViTConfig
import torch
import sys

# ============================================================================
# DEFAULT DESCRIPTIONS - Edit these to change what gets tokenized by default
# ============================================================================
DEFAULT_CLASSES = [
    "no object",        # Always keep this as the first item
    "orange mallet",    # Task 1: Orange hammer/mallet
    "rock pick hammer", # Task 2: Rock pick hammer  
    "water bottle",     # Task 3: Water bottle
]
# ============================================================================

def format_cpp_array(values: list, indent: str = "        ") -> str:
    """Format a list of integers as a C++ initializer list."""
    return indent + "{" + ", ".join(str(v) for v in values) + "}"

def main():
    # Parse command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == "-n" or sys.argv[1] == "--no-object":
            classes = ["no object"] + sys.argv[2:]
        elif sys.argv[1] in ["-h", "--help"]:
            print(__doc__)
            return
        else:
            classes = sys.argv[1:]
        print(f"Using custom descriptions: {classes}")
    else:
        classes = DEFAULT_CLASSES
        print(f"Using default descriptions: {classes}")
    
    model_name = "google/owlvit-base-patch32"
    processor = OwlViTProcessor.from_pretrained(model_name)

    # Move to GPU if available
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    example = processor(text=classes, return_tensors="pt")
    example = {k: v.to(device) if isinstance(v, torch.Tensor) else v for k, v in example.items()}

    print()
    print("=" * 60)
    print("Raw Tokens")
    print("=" * 60)
    print(f'Input IDs:\n{example["input_ids"]}')
    print()
    print(f'Attention Mask:\n{example["attention_mask"]}')
    
    # Generate C++ code
    print()
    print("=" * 60)
    print("C++ Code for ObjectDetector.cpp")
    print("=" * 60)
    
    input_ids = example["input_ids"].tolist()
    attention_mask = example["attention_mask"].tolist()
    
    class_names = ', '.join(f'"{c}"' for c in classes)
    input_ids_rows = [format_cpp_array(row) for row in input_ids]
    attention_rows = [format_cpp_array(row) for row in attention_mask]
    
    input_ids_str = ",\n".join(input_ids_rows)
    attention_str = ",\n".join(attention_rows)
    
    print(f'''
TaskConfig config;
config.class_names = {{{class_names}}};
// Tokens for: {classes}
config.input_ids = torch::tensor({{
{input_ids_str}
}}, torch::kInt64).to(device_);
config.attention_mask = torch::tensor({{
{attention_str}
}}, torch::kInt64).to(device_);
''')

if __name__ == "__main__":
    main()

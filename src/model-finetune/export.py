#!/usr/bin/env python3
"""Export fine-tuned OWL-ViT to TorchScript for C++ inference.

The exported model takes only pixel_values as input (text embeddings are baked in).
This avoids CUDA device mismatch issues with the text encoder's causal attention mask.

Usage:
    python export.py \
        --checkpoint runs/three_class_v5/final \
        --output ../object-detection/owlvit_finetune.pt
"""

import argparse
import json
from pathlib import Path

import torch
from transformers import OwlViTForObjectDetection, OwlViTProcessor


class OwlVitVisionOnly(torch.nn.Module):
    """Vision-only wrapper with pre-computed text embeddings.

    Forward signature: (pixel_values) → (logits, pred_boxes)
    Text embeddings are baked in as buffers so no text encoder runs at inference.
    """

    def __init__(self, model: OwlViTForObjectDetection, text_embeds, query_mask, box_bias):
        super().__init__()
        self.vision_model = model.owlvit.vision_model
        self.layer_norm = model.layer_norm
        self.class_head = model.class_head
        self.box_head = model.box_head
        self.register_buffer("text_embeds", text_embeds)
        self.register_buffer("query_mask", query_mask.unsqueeze(0))
        self.register_buffer("box_bias", box_bias)

    def forward(self, pixel_values: torch.Tensor):
        # Vision encode
        vision_out = self.vision_model(pixel_values=pixel_values)
        last_hidden = self.vision_model.post_layernorm(vision_out[0])

        # Class token * patch features (OWL-ViT specific)
        class_token = torch.broadcast_to(last_hidden[:, :1, :], last_hidden[:, :-1].shape)
        image_embeds = last_hidden[:, 1:, :] * class_token
        image_embeds = self.layer_norm(image_embeds)

        # Class prediction
        query_embeds = self.text_embeds.unsqueeze(0).expand(image_embeds.shape[0], -1, -1)
        pred_logits, _ = self.class_head(image_embeds, query_embeds, self.query_mask)

        # Box prediction
        pred_boxes = self.box_head(image_embeds) + self.box_bias
        pred_boxes = torch.sigmoid(pred_boxes)

        return pred_logits, pred_boxes


def main():
    parser = argparse.ArgumentParser(description="Export OWL-ViT to TorchScript")
    parser.add_argument("--checkpoint", required=True, help="HF checkpoint directory")
    parser.add_argument("--output", required=True, help="Output .pt path")
    parser.add_argument("--image-size", type=int, default=768)
    args = parser.parse_args()

    model_dir = Path(args.checkpoint)
    class_names_path = model_dir / "class_names.json"
    with open(class_names_path, "r", encoding="utf-8") as f:
        class_names = json.load(f)

    print(f"Classes: {class_names}")
    print(f"Loading model from {model_dir}...")

    model = OwlViTForObjectDetection.from_pretrained(model_dir).eval()
    processor = OwlViTProcessor.from_pretrained(model_dir)

    # Pre-compute text embeddings (baked into the exported model)
    tokenized = processor(text=[class_names], return_tensors="pt", padding=True)
    input_ids = tokenized["input_ids"]
    attention_mask = tokenized["attention_mask"]

    print("Pre-computing text embeddings...")
    with torch.no_grad():
        text_out = model.owlvit.text_model(input_ids=input_ids, attention_mask=attention_mask)
        text_embeds = model.owlvit.text_projection(text_out[1])

    query_mask = input_ids[..., 0] > 0
    box_bias = model.box_bias

    wrapped = OwlVitVisionOnly(model, text_embeds, query_mask, box_bias).eval()

    # Trace on CPU (safe for CUDA reload)
    pixel_values = torch.randn(1, 3, args.image_size, args.image_size)

    print("Tracing...")
    with torch.no_grad():
        traced = torch.jit.trace(wrapped, (pixel_values,), strict=False)

    output_pt = Path(args.output)
    output_pt.parent.mkdir(parents=True, exist_ok=True)
    traced.save(str(output_pt))

    import os
    size_mb = os.path.getsize(output_pt) / 1024 / 1024
    print(f"Saved: {output_pt} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()

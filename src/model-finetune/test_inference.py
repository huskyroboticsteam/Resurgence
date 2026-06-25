#!/usr/bin/env python3
"""Quick inference test on example images using the fine-tuned model.

Usage:
    python test_inference.py --model-dir runs/three_class_v2/best --images example/
"""

import argparse
import json
from pathlib import Path

import torch
from PIL import Image, ImageDraw, ImageFont
from transformers import OwlViTForObjectDetection, OwlViTProcessor

COLORS = ["red", "lime", "cyan", "yellow", "magenta", "orange"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--images", required=True, help="Image file or directory")
    parser.add_argument("--threshold", type=float, default=0.3)
    parser.add_argument("--output-dir", default="test_results")
    args = parser.parse_args()

    model_dir = Path(args.model_dir)
    with open(model_dir / "class_names.json") as f:
        class_names = json.load(f)

    print(f"Classes: {class_names}")

    processor = OwlViTProcessor.from_pretrained(model_dir)
    model = OwlViTForObjectDetection.from_pretrained(model_dir).eval()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)

    # Use ALL class names (including "no object") — must match training
    query_names = class_names

    images_path = Path(args.images)
    if images_path.is_dir():
        image_files = sorted(
            f for f in images_path.iterdir()
            if f.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp", ".bmp"}
        )
    else:
        image_files = [images_path]

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for img_path in image_files:
        print(f"\n{'=' * 50}")
        print(f"Image: {img_path.name}")

        image = Image.open(img_path).convert("RGB")
        w, h = image.size

        inputs = processor(images=image, text=query_names, return_tensors="pt", padding=True)
        inputs = {k: v.to(device) for k, v in inputs.items()}

        with torch.no_grad():
            outputs = model(**inputs)

        logits = outputs.logits[0]  # [Q, C]
        boxes = outputs.pred_boxes[0]  # [Q, 4]

        # Softmax over classes (trained with cross_entropy)
        probs = logits.softmax(dim=-1)
        max_scores, max_classes = probs.max(dim=-1)

        # Filter: above threshold AND not "no object" (class 0)
        mask = (max_scores > args.threshold) & (max_classes > 0)
        det_scores = max_scores[mask]
        det_classes = max_classes[mask]
        det_boxes = boxes[mask]

        # Draw
        draw = ImageDraw.Draw(image)
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
        except Exception:
            font = ImageFont.load_default()

        if len(det_scores) == 0:
            print("  No detections above threshold.")
        else:
            # Sort by score descending
            order = det_scores.argsort(descending=True)
            for i in order:
                cls_idx = det_classes[i].item()
                score = det_scores[i].item()
                cx, cy, bw, bh = det_boxes[i].tolist()

                x1 = int((cx - bw / 2) * w)
                y1 = int((cy - bh / 2) * h)
                x2 = int((cx + bw / 2) * w)
                y2 = int((cy + bh / 2) * h)

                name = class_names[cls_idx] if cls_idx < len(class_names) else f"class_{cls_idx}"
                color = COLORS[(cls_idx - 1) % len(COLORS)]  # -1 to skip "no object" color

                print(f"  {name}: {score:.3f}  [{x1},{y1},{x2},{y2}]")
                draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
                draw.text((x1 + 2, y1 - 18), f"{name} {score:.2f}", fill=color, font=font)

        out_path = out_dir / f"{img_path.stem}_det.jpg"
        image.save(out_path, quality=95)
        print(f"  Saved: {out_path}")

    print(f"\nDone! Results in {out_dir}/")


if __name__ == "__main__":
    main()

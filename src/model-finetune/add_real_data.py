#!/usr/bin/env python3
"""Add real-world photos to existing COCO datasets with Grounding DINO auto-annotation.

Usage:
    python add_real_data.py --input /path/to/photos --class orange_mallet
    python add_real_data.py --input /path/to/photos --class orange_mallet --preview
    python add_real_data.py --input /path/to/photos --class orange_mallet --box-threshold 0.2
"""

import argparse
import json
import shutil
from pathlib import Path

import torch
from PIL import Image
from torchvision.ops import nms
from tqdm import tqdm
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

CLASS_CONFIG = {
    "orange_mallet": {"prompt": "orange dead blow hammer."},
    "rock_pick_hammer": {"prompt": "rock pick hammer."},
    "water_bottle": {"prompt": "insulated water bottle."},
}

DATASET_BASE = Path(__file__).parent / "datasets" / "web_coco"
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}
GROUNDING_MODEL = "IDEA-Research/grounding-dino-base"


def annotate_images(image_paths, prompt, processor, model, device, box_threshold):
    results = []
    for img_path in tqdm(image_paths, desc="Annotating"):
        try:
            image = Image.open(img_path).convert("RGB")
        except Exception:
            print(f"  Skipping (can't open): {img_path.name}")
            continue

        w, h = image.size
        inputs = processor(images=image, text=prompt, return_tensors="pt").to(device)
        with torch.no_grad():
            outputs = model(**inputs)

        r = processor.post_process_grounded_object_detection(
            outputs, inputs["input_ids"],
            threshold=box_threshold, text_threshold=box_threshold,
            target_sizes=[(h, w)],
        )[0]

        boxes_xyxy, scores = r["boxes"], r["scores"]
        if boxes_xyxy.numel() == 0:
            continue

        keep = nms(boxes_xyxy, scores, 0.5)
        boxes_xyxy = boxes_xyxy[keep]

        coco_boxes = []
        for box in boxes_xyxy.cpu().tolist():
            x1, y1, x2, y2 = box
            bw, bh = x2 - x1, y2 - y1
            if bw <= 2 or bh <= 2:
                continue
            coco_boxes.append([
                round(max(0, x1), 2), round(max(0, y1), 2),
                round(min(bw, w - x1), 2), round(min(bh, h - y1), 2),
            ])

        if coco_boxes:
            results.append({"path": img_path, "width": w, "height": h, "boxes": coco_boxes})

    return results


def preview_annotations(results, output_dir):
    import cv2
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for entry in results:
        img = cv2.imread(str(entry["path"]))
        for box in entry["boxes"]:
            x, y, w, h = [int(v) for v in box]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imwrite(str(output_dir / entry["path"].name), img)
    print(f"\nPreview saved to: {output_dir}")
    print("Check the images, then re-run without --preview to merge.")


def merge_into_dataset(results, class_name, val_ratio=0.1):
    """Split into train/val and merge into existing dataset."""
    import random
    random.shuffle(results)
    n_val = max(1, int(len(results) * val_ratio))
    splits = {"val": results[:n_val], "train": results[n_val:]}

    for split, entries in splits.items():
        dataset_dir = DATASET_BASE / class_name / split
        data_dir = dataset_dir / "data"
        labels_path = dataset_dir / "labels.json"
        data_dir.mkdir(parents=True, exist_ok=True)

        if labels_path.exists():
            with open(labels_path) as f:
                coco = json.load(f)
        else:
            coco = {
                "images": [], "annotations": [],
                "categories": [{"id": 1, "name": class_name, "supercategory": None}],
            }

        img_id = max((img["id"] for img in coco["images"]), default=0)
        ann_id = max((ann["id"] for ann in coco["annotations"]), default=0)

        for entry in entries:
            img_id += 1
            src = entry["path"]
            dest = data_dir / f"{img_id:06d}{src.suffix.lower()}"
            shutil.copy2(src, dest)
            coco["images"].append({
                "id": img_id, "file_name": dest.name,
                "height": entry["height"], "width": entry["width"],
            })
            for box in entry["boxes"]:
                ann_id += 1
                bx, by, bw, bh = box
                coco["annotations"].append({
                    "id": ann_id, "image_id": img_id, "category_id": 1,
                    "bbox": box, "area": round(bw * bh, 2), "iscrowd": 0,
                })

        with open(labels_path, "w") as f:
            json.dump(coco, f, indent=2)

        print(f"  {split}: added {len(entries)} images (total: {len(coco['images'])})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--class", dest="cls", required=True, choices=list(CLASS_CONFIG.keys()))
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--box-threshold", type=float, default=0.25)
    parser.add_argument("--val-ratio", type=float, default=0.1)
    args = parser.parse_args()

    input_dir = Path(args.input)
    image_paths = sorted(f for f in input_dir.iterdir()
                         if f.is_file() and f.suffix.lower() in IMAGE_EXTENSIONS)
    if not image_paths:
        print(f"No images found in {input_dir}")
        return

    print(f"Found {len(image_paths)} images  |  class: {args.cls}")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Loading Grounding DINO... (device: {device})")
    processor = AutoProcessor.from_pretrained(GROUNDING_MODEL)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(GROUNDING_MODEL).to(device)
    model.eval()

    results = annotate_images(
        image_paths, CLASS_CONFIG[args.cls]["prompt"],
        processor, model, device, args.box_threshold,
    )
    print(f"\n{len(results)}/{len(image_paths)} images annotated")

    if not results:
        print("Nothing to add.")
        return

    if args.preview:
        preview_annotations(results, input_dir / "_preview")
    else:
        print(f"\nMerging into dataset (val_ratio={args.val_ratio})...")
        merge_into_dataset(results, args.cls, args.val_ratio)
        print("\nDone! Run train.py to retrain.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Download web images and auto-annotate with Grounding DINO → COCO format.

Run once per class:
    python collect_data.py \
        --query "orange dead blow hammer" "dead blow hammer orange" \
        --prompt "orange dead blow hammer." \
        --category "Hammer" \
        --output datasets/web_coco/orange_hammer \
        --max-images 100

Or use built-in presets:
    python collect_data.py --preset orange_hammer
    python collect_data.py --preset rock_pick
    python collect_data.py --preset water_bottle
    python collect_data.py --preset all
"""

import argparse
import hashlib
import json
import logging
import random
from pathlib import Path

import torch
from PIL import Image
from torchvision.ops import nms
from tqdm import tqdm
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

# ── Presets ──────────────────────────────────────────────────────────────

PRESETS = {
    "orange_hammer": {
        "queries": [
            "orange dead blow hammer",
            "dead blow hammer orange",
            "orange dead blow mallet",
            "orange dead blow hammer tool",
            "dead blow mallet orange rubber",
        ],
        "prompt": "orange dead blow hammer.",
        "category": "Hammer",
    },
    "rock_pick": {
        "queries": [
            "Estwing rock pick",
            "geological rock pick hammer",
            "rock pick hammer pointed tip",
            "geology hammer rock pick",
            "rock pick prospecting hammer",
        ],
        "prompt": "rock pick hammer.",
        "category": "Hammer",
    },
    "water_bottle": {
        "queries": [
            "insulated stainless steel water bottle",
            "black insulated water bottle",
            "insulated water bottle flip lid",
            "stainless steel sport water bottle",
            "thermos water bottle",
        ],
        "prompt": "insulated water bottle.",
        "category": "Bottle",
    },
}

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}
GROUNDING_MODEL = "IDEA-Research/grounding-dino-base"
BOX_THRESHOLD = 0.25
TEXT_THRESHOLD = 0.25
NMS_THRESHOLD = 0.5
VAL_FRACTION = 0.15
MIN_IMAGE_SIZE = 200
SEED = 42


# ── Download ─────────────────────────────────────────────────────────────


def download_images(queries: list, max_images: int, out_dir: Path) -> int:
    from icrawler.builtin import BingImageCrawler

    out_dir.mkdir(parents=True, exist_ok=True)
    per_query = max(1, max_images // len(queries))

    for i, query in enumerate(queries):
        print(f"  [{i + 1}/{len(queries)}] '{query}' (max {per_query})")
        crawler = BingImageCrawler(
            storage={"root_dir": str(out_dir)}, log_level=logging.WARNING
        )
        crawler.crawl(keyword=query, max_num=per_query)

    # Validate & deduplicate
    removed = 0
    for f in sorted(out_dir.iterdir()):
        if not f.is_file() or f.suffix.lower() not in IMAGE_EXTENSIONS:
            if f.is_file():
                f.unlink()
                removed += 1
            continue
        try:
            with Image.open(f) as im:
                if im.size[0] < MIN_IMAGE_SIZE or im.size[1] < MIN_IMAGE_SIZE:
                    f.unlink()
                    removed += 1
                    continue
                im.verify()
        except Exception:
            f.unlink()
            removed += 1

    seen, dupes = {}, 0
    for f in sorted(out_dir.iterdir()):
        if not f.is_file():
            continue
        h = hashlib.md5(f.read_bytes()).hexdigest()
        if h in seen:
            f.unlink()
            dupes += 1
        else:
            seen[h] = f

    total = sum(1 for f in out_dir.iterdir() if f.is_file())
    print(f"  → {total} unique images (removed {removed} invalid, {dupes} dupes)")
    return total


# ── Annotate ─────────────────────────────────────────────────────────────


def annotate_images(image_dir: Path, prompt: str, processor, model, device: str) -> list:
    image_paths = sorted(
        f for f in image_dir.iterdir() if f.is_file() and f.suffix.lower() in IMAGE_EXTENSIONS
    )
    results = []

    for img_path in tqdm(image_paths, desc="  Annotating"):
        try:
            image = Image.open(img_path).convert("RGB")
        except Exception:
            continue

        w, h = image.size
        inputs = processor(images=image, text=prompt, return_tensors="pt").to(device)

        with torch.no_grad():
            outputs = model(**inputs)

        r = processor.post_process_grounded_object_detection(
            outputs, inputs["input_ids"],
            threshold=BOX_THRESHOLD, text_threshold=TEXT_THRESHOLD,
            target_sizes=[(h, w)],
        )[0]

        boxes_xyxy, scores = r["boxes"], r["scores"]
        if boxes_xyxy.numel() == 0:
            continue

        keep = nms(boxes_xyxy, scores, NMS_THRESHOLD)
        boxes_xyxy = boxes_xyxy[keep]
        scores = scores[keep]

        # Convert to COCO [x, y, w, h]
        coco_boxes = []
        for box, score in zip(boxes_xyxy.cpu().tolist(), scores.cpu().tolist()):
            x1, y1, x2, y2 = box
            bw, bh = x2 - x1, y2 - y1
            if bw <= 2 or bh <= 2:
                continue
            x1 = max(0, min(x1, w))
            y1 = max(0, min(y1, h))
            coco_boxes.append([round(x1, 2), round(y1, 2), round(min(bw, w - x1), 2), round(min(bh, h - y1), 2)])

        if coco_boxes:
            results.append({"path": img_path, "width": w, "height": h, "boxes": coco_boxes})

    print(f"  → {len(results)}/{len(image_paths)} images with detections")
    return results


# ── Build COCO ───────────────────────────────────────────────────────────


def build_coco(entries: list, category_name: str, output_dir: Path):
    import shutil

    data_dir = output_dir / "data"
    data_dir.mkdir(parents=True, exist_ok=True)

    images_list, annotations_list = [], []
    img_id = ann_id = 0

    for entry in entries:
        img_id += 1
        src = entry["path"]
        dest = data_dir / f"{img_id:06d}{src.suffix.lower()}"
        shutil.copy2(src, dest)

        images_list.append({
            "id": img_id, "file_name": dest.name,
            "height": entry["height"], "width": entry["width"],
        })

        for box in entry["boxes"]:
            ann_id += 1
            bx, by, bw, bh = box
            annotations_list.append({
                "id": ann_id, "image_id": img_id, "category_id": 1,
                "bbox": box, "area": round(bw * bh, 2), "iscrowd": 0,
            })

    coco = {
        "images": images_list,
        "annotations": annotations_list,
        "categories": [{"id": 1, "name": category_name, "supercategory": None}],
    }
    with open(output_dir / "labels.json", "w", encoding="utf-8") as f:
        json.dump(coco, f, indent=2, ensure_ascii=False)

    return len(images_list), len(annotations_list)


# ── Main ─────────────────────────────────────────────────────────────────


def process_one(queries, prompt, category, output_dir, max_images, processor, model, device):
    raw_dir = output_dir / "_raw"

    # Download
    print("\n[1/3] Downloading...")
    download_images(queries, max_images, raw_dir)

    # Annotate
    print("\n[2/3] Annotating with Grounding DINO...")
    results = annotate_images(raw_dir, prompt, processor, model, device)
    if not results:
        print("  No detections. Skipping.")
        return

    # Split & save
    print("\n[3/3] Building COCO dataset...")
    random.seed(SEED)
    random.shuffle(results)
    n_val = max(1, int(len(results) * VAL_FRACTION))

    n_img, n_ann = build_coco(results[n_val:], category, output_dir / "train")
    print(f"  train: {n_img} images, {n_ann} annotations")

    n_img, n_ann = build_coco(results[:n_val], category, output_dir / "val")
    print(f"  val:   {n_img} images, {n_ann} annotations")

    print(f"  Output: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="Download + auto-annotate → COCO dataset")
    parser.add_argument("--preset", choices=list(PRESETS.keys()) + ["all"], help="Use built-in preset")
    parser.add_argument("--query", nargs="+", help="Search queries (if not using preset)")
    parser.add_argument("--prompt", help="Grounding DINO prompt (if not using preset)")
    parser.add_argument("--category", help="COCO category name (if not using preset)")
    parser.add_argument("--output", help="Output directory (if not using preset)")
    parser.add_argument("--max-images", type=int, default=100)
    parser.add_argument("--base-dir", default="datasets", help="Base dir for presets")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Loading Grounding DINO... (device: {device})")
    gd_processor = AutoProcessor.from_pretrained(GROUNDING_MODEL)
    gd_model = AutoModelForZeroShotObjectDetection.from_pretrained(GROUNDING_MODEL).to(device)
    gd_model.eval()

    if args.preset:
        keys = list(PRESETS.keys()) if args.preset == "all" else [args.preset]
        base = Path(args.base_dir)
        for key in keys:
            cfg = PRESETS[key]
            print(f"\n{'=' * 60}\n{key}\n{'=' * 60}")
            process_one(
                cfg["queries"], cfg["prompt"], cfg["category"],
                base / "web_coco" / key, args.max_images,
                gd_processor, gd_model, device,
            )
    else:
        if not all([args.query, args.prompt, args.category, args.output]):
            parser.error("Provide --query, --prompt, --category, --output (or use --preset)")
        process_one(
            args.query, args.prompt, args.category,
            Path(args.output), args.max_images,
            gd_processor, gd_model, device,
        )

    print("\nAll done!")


if __name__ == "__main__":
    main()

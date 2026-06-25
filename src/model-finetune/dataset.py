"""COCO-format dataset for OWL-ViT fine-tuning.

Each dataset instance maps all annotations to a single class index,
enabling multi-class training via ConcatDataset.
"""

import json
import random
from pathlib import Path
from typing import Dict, List, Tuple

import torch
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms as T


class CocoDetectionDataset(Dataset):
    def __init__(
        self,
        images_dir: str,
        annotations_json: str,
        class_idx: int,
        augment: bool = False,
    ):
        self.images_dir = Path(images_dir)
        self.class_idx = class_idx
        self.augment = augment
        self.color_jitter = T.ColorJitter(
            brightness=0.3, contrast=0.3, saturation=0.3, hue=0.05
        )

        with open(annotations_json, "r", encoding="utf-8") as f:
            coco = json.load(f)

        self.images = coco.get("images", [])
        self.anns_by_image: Dict[int, List[dict]] = {}
        for ann in coco.get("annotations", []):
            self.anns_by_image.setdefault(ann["image_id"], []).append(ann)

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> Tuple[Image.Image, Dict[str, torch.Tensor]]:
        img_info = self.images[idx]
        image = Image.open(self.images_dir / img_info["file_name"]).convert("RGB")

        anns = self.anns_by_image.get(img_info["id"], [])
        w, h = img_info["width"], img_info["height"]

        boxes = []
        for ann in anns:
            if ann.get("iscrowd", 0):
                continue
            x, y, bw, bh = ann["bbox"]
            if bw <= 1 or bh <= 1:
                continue
            # Convert COCO [x,y,w,h] → normalized [cx,cy,w,h]
            cx = min(max((x + bw / 2) / w, 0.0), 1.0)
            cy = min(max((y + bh / 2) / h, 0.0), 1.0)
            nw = min(max(bw / w, 0.0), 1.0)
            nh = min(max(bh / h, 0.0), 1.0)
            boxes.append([cx, cy, nw, nh])

        if boxes:
            labels = {
                "class_labels": torch.full((len(boxes),), self.class_idx, dtype=torch.long),
                "boxes": torch.tensor(boxes, dtype=torch.float32),
            }
        else:
            labels = {
                "class_labels": torch.zeros((0,), dtype=torch.long),
                "boxes": torch.zeros((0, 4), dtype=torch.float32),
            }

        if self.augment:
            image, labels = self._augment(image, labels)

        return image, labels

    def _augment(
        self, image: Image.Image, labels: Dict[str, torch.Tensor]
    ) -> Tuple[Image.Image, Dict[str, torch.Tensor]]:
        # Horizontal flip (50%)
        if random.random() < 0.5:
            image = image.transpose(Image.FLIP_LEFT_RIGHT)
            if labels["boxes"].numel() > 0:
                boxes = labels["boxes"].clone()
                boxes[:, 0] = 1.0 - boxes[:, 0]  # mirror cx
                labels["boxes"] = boxes

        # Color jitter (80%)
        if random.random() < 0.8:
            image = self.color_jitter(image)

        return image, labels

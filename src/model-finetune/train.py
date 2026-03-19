#!/usr/bin/env python3
"""Fine-tune OWL-ViT detection heads on custom COCO datasets.

Freezes the CLIP backbone and only trains class_head + box_head + layer_norm.
Includes LR warmup, cosine decay, gradient clipping, and balanced sampling.

Usage:
    python train.py \
        --datasets "orange mallet=datasets/web_coco/orange_hammer" \
                   "rock pick hammer=datasets/web_coco/rock_pick" \
                   "water bottle=datasets/web_coco/water_bottle" \
        --output runs/three_class_v2 \
        --epochs 30
"""

import argparse
import json
import random
from collections import Counter
from pathlib import Path
from typing import Dict, List, Tuple

import torch
import torch.nn.functional as F
from scipy.optimize import linear_sum_assignment
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR, LinearLR, SequentialLR
from torch.utils.data import ConcatDataset, DataLoader, WeightedRandomSampler
from tqdm import tqdm
from transformers import OwlViTForObjectDetection, OwlViTProcessor

from dataset import CocoDetectionDataset


# ── Geometry helpers ─────────────────────────────────────────────────────


def box_cxcywh_to_xyxy(boxes: torch.Tensor) -> torch.Tensor:
    cx, cy, w, h = boxes.unbind(-1)
    return torch.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], dim=-1)


def generalized_iou(boxes1: torch.Tensor, boxes2: torch.Tensor) -> torch.Tensor:
    """Pairwise GIoU between two sets of xyxy boxes. Returns [N, M] matrix."""
    if boxes1.numel() == 0 or boxes2.numel() == 0:
        return torch.zeros((boxes1.size(0), boxes2.size(0)), device=boxes1.device)

    area1 = (boxes1[:, 2:] - boxes1[:, :2]).clamp(min=0).prod(-1)
    area2 = (boxes2[:, 2:] - boxes2[:, :2]).clamp(min=0).prod(-1)

    lt = torch.max(boxes1[:, None, :2], boxes2[:, :2])
    rb = torch.min(boxes1[:, None, 2:], boxes2[:, 2:])
    inter = (rb - lt).clamp(min=0).prod(-1)

    union = area1[:, None] + area2 - inter
    iou = inter / union.clamp(min=1e-6)

    c_lt = torch.min(boxes1[:, None, :2], boxes2[:, :2])
    c_rb = torch.max(boxes1[:, None, 2:], boxes2[:, 2:])
    c_area = (c_rb - c_lt).clamp(min=0).prod(-1)

    return iou - (c_area - union) / c_area.clamp(min=1e-6)


# ── Hungarian matching ───────────────────────────────────────────────────


def hungarian_match(
    pred_logits: torch.Tensor,
    pred_boxes: torch.Tensor,
    tgt_classes: torch.Tensor,
    tgt_boxes: torch.Tensor,
) -> Tuple[torch.Tensor, torch.Tensor]:
    if tgt_boxes.size(0) == 0:
        empty = torch.zeros((0,), dtype=torch.long, device=pred_logits.device)
        return empty, empty

    probs = pred_logits.softmax(-1)
    cost_cls = -probs[:, tgt_classes]
    cost_l1 = torch.cdist(pred_boxes, tgt_boxes, p=1)
    cost_giou = -generalized_iou(
        box_cxcywh_to_xyxy(pred_boxes), box_cxcywh_to_xyxy(tgt_boxes)
    )

    cost = cost_cls + 5.0 * cost_l1 + 2.0 * cost_giou
    row, col = linear_sum_assignment(cost.detach().cpu().numpy())

    return (
        torch.as_tensor(row, dtype=torch.long, device=pred_logits.device),
        torch.as_tensor(col, dtype=torch.long, device=pred_logits.device),
    )


# ── Detection loss ───────────────────────────────────────────────────────


def compute_loss(
    outputs, labels: List[Dict[str, torch.Tensor]], num_classes: int
) -> Tuple[torch.Tensor, dict]:
    logits = outputs.logits  # [B, Q, C]
    pred_boxes = outputs.pred_boxes  # [B, Q, 4]
    device = logits.device
    B, Q, _ = logits.shape

    # Default all queries to "no object" (class 0)
    target_classes = torch.zeros((B, Q), dtype=torch.long, device=device)
    matched_pred, matched_tgt = [], []

    for b in range(B):
        tgt_cls = labels[b]["class_labels"].to(device)
        tgt_box = labels[b]["boxes"].to(device)
        src_idx, tgt_idx = hungarian_match(logits[b], pred_boxes[b], tgt_cls, tgt_box)
        if src_idx.numel() > 0:
            target_classes[b, src_idx] = tgt_cls[tgt_idx]
            matched_pred.append(pred_boxes[b][src_idx])
            matched_tgt.append(tgt_box[tgt_idx])

    loss_cls = F.cross_entropy(logits.reshape(-1, num_classes), target_classes.reshape(-1))

    if matched_pred:
        pred_cat = torch.cat(matched_pred)
        tgt_cat = torch.cat(matched_tgt)
        loss_l1 = F.l1_loss(pred_cat, tgt_cat)
        giou_vals = torch.diag(
            generalized_iou(box_cxcywh_to_xyxy(pred_cat), box_cxcywh_to_xyxy(tgt_cat))
        )
        loss_giou = (1.0 - giou_vals).mean()
    else:
        loss_l1 = torch.tensor(0.0, device=device)
        loss_giou = torch.tensor(0.0, device=device)

    total = loss_cls + 5.0 * loss_l1 + 2.0 * loss_giou

    return total, {
        "cls": loss_cls.item(),
        "l1": loss_l1.item(),
        "giou": loss_giou.item(),
    }


# ── Training / evaluation ───────────────────────────────────────────────


def train_one_epoch(model, loader, optimizer, scheduler, device, num_classes, max_grad_norm):
    model.train()
    total_loss = 0.0
    steps = 0

    for batch in tqdm(loader, desc="  train", leave=False):
        outputs = model(
            pixel_values=batch["pixel_values"].to(device),
            input_ids=batch["input_ids"].to(device),
            attention_mask=batch["attention_mask"].to(device),
            return_dict=True,
        )
        loss, parts = compute_loss(outputs, batch["labels"], num_classes)

        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(
            [p for p in model.parameters() if p.requires_grad], max_grad_norm
        )
        optimizer.step()
        scheduler.step()

        total_loss += loss.item()
        steps += 1

    return total_loss / max(1, steps)


@torch.no_grad()
def evaluate(model, loader, device, num_classes):
    if loader is None:
        return None
    model.eval()
    total_loss = 0.0
    steps = 0

    for batch in tqdm(loader, desc="  val", leave=False):
        outputs = model(
            pixel_values=batch["pixel_values"].to(device),
            input_ids=batch["input_ids"].to(device),
            attention_mask=batch["attention_mask"].to(device),
            return_dict=True,
        )
        loss, _ = compute_loss(outputs, batch["labels"], num_classes)
        total_loss += loss.item()
        steps += 1

    return total_loss / max(1, steps)


# ── Checkpoint saving ────────────────────────────────────────────────────


def save_checkpoint(model, processor, class_names, path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    model.save_pretrained(path)
    processor.save_pretrained(path)
    with open(path / "class_names.json", "w", encoding="utf-8") as f:
        json.dump(class_names, f, indent=2)


# ── Main ─────────────────────────────────────────────────────────────────


def parse_dataset_arg(s: str) -> Tuple[str, Path]:
    """Parse 'class name=path' into (name, path)."""
    name, path = s.split("=", 1)
    return name.strip(), Path(path.strip())


def main():
    parser = argparse.ArgumentParser(description="Fine-tune OWL-ViT detection heads")
    parser.add_argument(
        "--datasets", nargs="+", required=True,
        help="'class name=path' pairs, e.g. 'orange mallet=datasets/web_coco/orange_hammer'",
    )
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--model", default="google/owlvit-base-patch32")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--warmup-epochs", type=int, default=3)
    parser.add_argument("--max-grad-norm", type=float, default=0.1)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    # ── Parse class sources ──────────────────────────────────────────────
    sources = [parse_dataset_arg(s) for s in args.datasets]
    class_names = ["no object"] + [name for name, _ in sources]
    num_classes = len(class_names)

    print(f"Classes ({num_classes}): {class_names}")
    with open(output / "class_names.json", "w", encoding="utf-8") as f:
        json.dump(class_names, f, indent=2)

    # ── Load model & freeze backbone ─────────────────────────────────────
    processor = OwlViTProcessor.from_pretrained(args.model)
    model = OwlViTForObjectDetection.from_pretrained(args.model)

    for p in model.parameters():
        p.requires_grad = False
    for name, p in model.named_parameters():
        if any(h in name for h in ("class_head", "box_head", "layer_norm")):
            p.requires_grad = True

    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total = sum(p.numel() for p in model.parameters())
    print(f"Trainable: {trainable:,} / {total:,} ({100 * trainable / total:.1f}%)")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)

    # ── Build datasets ───────────────────────────────────────────────────
    train_datasets, val_datasets = [], []
    for name, path in sources:
        class_idx = class_names.index(name)

        train_dir = path / "train"
        if train_dir.exists() and (train_dir / "labels.json").exists():
            ds = CocoDetectionDataset(
                train_dir / "data", train_dir / "labels.json", class_idx, augment=True
            )
            train_datasets.append(ds)
            print(f"  {name} train: {len(ds)} images")

        val_dir = path / "val"
        if val_dir.exists() and (val_dir / "labels.json").exists():
            ds = CocoDetectionDataset(
                val_dir / "data", val_dir / "labels.json", class_idx
            )
            val_datasets.append(ds)
            print(f"  {name} val:   {len(ds)} images")

    train_dataset = ConcatDataset(train_datasets)
    val_dataset = ConcatDataset(val_datasets) if val_datasets else None

    # ── Balanced sampling ────────────────────────────────────────────────
    class_per_sample = []
    for ds in train_datasets:
        class_per_sample.extend([ds.class_idx] * len(ds))

    counts = Counter(class_per_sample)
    n_total = len(class_per_sample)
    weights = [n_total / max(1, counts[c]) for c in class_per_sample]
    sampler = WeightedRandomSampler(weights, num_samples=n_total, replacement=True)

    print("Class balance:")
    for idx, cnt in sorted(counts.items()):
        print(f"  {class_names[idx]}: {cnt} images ({n_total / max(1, cnt):.1f}x)")

    # ── Collate function ─────────────────────────────────────────────────
    def collate_fn(batch):
        images, labels = zip(*batch)
        texts = [class_names] * len(images)
        enc = processor(images=list(images), text=texts, return_tensors="pt", padding=True)
        return {
            "pixel_values": enc["pixel_values"],
            "input_ids": enc["input_ids"],
            "attention_mask": enc["attention_mask"],
            "labels": list(labels),
        }

    train_loader = DataLoader(
        train_dataset, batch_size=args.batch_size, sampler=sampler,
        num_workers=args.workers, collate_fn=collate_fn, pin_memory=True,
    )
    val_loader = None
    if val_dataset:
        val_loader = DataLoader(
            val_dataset, batch_size=args.batch_size, shuffle=False,
            num_workers=args.workers, collate_fn=collate_fn, pin_memory=True,
        )

    # ── Optimizer & scheduler ────────────────────────────────────────────
    trainable_params = [p for p in model.parameters() if p.requires_grad]
    optimizer = AdamW(trainable_params, lr=args.lr, weight_decay=0.01)

    steps_per_epoch = len(train_loader)
    warmup_steps = args.warmup_epochs * steps_per_epoch
    total_steps = args.epochs * steps_per_epoch

    warmup = LinearLR(optimizer, start_factor=0.01, total_iters=warmup_steps)
    cosine = CosineAnnealingLR(optimizer, T_max=max(1, total_steps - warmup_steps))
    scheduler = SequentialLR(optimizer, [warmup, cosine], milestones=[warmup_steps])

    print(f"\nTraining: {args.epochs} epochs, {steps_per_epoch} steps/epoch")
    print(f"LR: {args.lr} with {args.warmup_epochs}-epoch warmup + cosine decay")
    print(f"Grad clip: {args.max_grad_norm}\n")

    # ── Training loop ────────────────────────────────────────────────────
    best_val = float("inf")

    for epoch in range(args.epochs):
        train_loss = train_one_epoch(
            model, train_loader, optimizer, scheduler, device, num_classes, args.max_grad_norm
        )
        val_loss = evaluate(model, val_loader, device, num_classes)

        line = f"epoch {epoch + 1:2d}/{args.epochs}  train={train_loss:.4f}"
        if val_loss is not None:
            line += f"  val={val_loss:.4f}"
            if epoch >= args.warmup_epochs and val_loss < best_val:
                best_val = val_loss
                line += "  ★ best"
                save_checkpoint(model, processor, class_names, output / "best")
        print(line)

        # Save periodic checkpoints
        if (epoch + 1) % 5 == 0:
            save_checkpoint(model, processor, class_names, output / f"epoch-{epoch + 1}")

    # Save final
    save_checkpoint(model, processor, class_names, output / "final")
    print(f"\nDone. Best val={best_val:.4f}. Output: {output}")


if __name__ == "__main__":
    main()

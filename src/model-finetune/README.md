# OWL-ViT Fine-tune

Fine-tune OWL-ViT for three object classes used in the HSR robot:
- **orange mallet** (class 1)
- **rock pick hammer** (class 2)
- **water bottle** (class 3)

The trained model exports as a TorchScript file (`owlvit_finetune.pt`) used by the C++ object detection node.

---

## Setup

```bash
conda activate hsr
cd src/model-finetune
pip install -r requirements.txt
```

---

## Complete Workflow

### Step 1 — Capture real-world photos

Connect the RealSense camera, then run:

```bash
python capture.py
```

Controls:
- `Space` — take a photo
- `1` / `2` / `3` — switch between orange_mallet / rock_pick_hammer / water_bottle
- `q` — quit

Photos are saved to `captured_images/<class_name>/`. Aim for **200+ photos per class** at different angles, distances, and lighting conditions.

---

### Step 2 — Add photos to the dataset

This annotates the photos automatically using Grounding DINO and merges them into the training dataset.

```bash
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet
python add_real_data.py --input captured_images/rock_pick_hammer --class rock_pick_hammer
python add_real_data.py --input captured_images/water_bottle --class water_bottle
```

To preview annotations before merging (generates images with bounding boxes drawn):

```bash
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet --preview
```

Check the `_preview/` folder inside your input directory, then run without `--preview` to merge.

If detections are missing or wrong, lower the threshold (default is 0.25):

```bash
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet --box-threshold 0.15
```

---

### Step 3 — Train

Replace `v7` with the next version number each time you retrain.

```bash
python train.py \
  --datasets "orange mallet=datasets/web_coco/orange_mallet" \
             "rock pick hammer=datasets/web_coco/rock_pick_hammer" \
             "water bottle=datasets/web_coco/water_bottle" \
  --output runs/three_class_v7 \
  --epochs 30
```

Training saves checkpoints every 5 epochs under `runs/three_class_v7/`. The `best` checkpoint is the one with the lowest validation loss after warmup; `final` is the last epoch.

---

### Step 4 — Export to TorchScript

```bash
python export.py \
  --checkpoint runs/three_class_v7/final \
  --output ../object-detection/owlvit_finetune.pt
```

This bakes the text embeddings into the model so only the vision encoder runs at inference. The output file is ~342 MB.

---

### Step 5 — Test

Delete the old model file so it doesn't load the stale cached version:

```bash
rm src/object-detection/owlvit_finetune.pt
```

Build and run:

```bash
cd build
cmake --build . --target realsense_test -j$(nproc)
./object-detection/realsense_test
```

Controls:
- `1` / `2` / `3` — detect orange mallet / rock pick hammer / water bottle
- `4` — detect all classes
- `+` / `-` — raise or lower confidence threshold
- `q` — quit

---

### Step 6 — Upload to HuggingFace (optional)

After confirming the model works, upload to `thomas0829/OWL-ViT_Finetune` so others can pull it:

```bash
python -c "
from huggingface_hub import HfApi
HfApi().upload_file(
    path_or_fileobj='../object-detection/owlvit_finetune.pt',
    path_in_repo='owlvit_finetune.pt',
    repo_id='thomas0829/OWL-ViT_Finetune',
)
"
```

---

## Dataset Structure

```
datasets/web_coco/
├── orange_mallet/
│   ├── train/
│   │   ├── data/          # image files (000001.jpg, ...)
│   │   └── labels.json    # COCO format annotations
│   └── val/
│       ├── data/
│       └── labels.json
├── rock_pick_hammer/
└── water_bottle/
```

## Files

| File | Description |
|------|-------------|
| `train.py` | Fine-tune OWL-ViT detection heads (frozen backbone) |
| `export.py` | Export trained checkpoint to TorchScript |
| `dataset.py` | COCO dataset loader |
| `capture.py` | Capture images from RealSense camera |
| `add_real_data.py` | Auto-annotate photos and merge into dataset |
| `collect_data.py` | Download web images and auto-annotate |

# OWL-ViT Fine-tune

Fine-tune OWL-ViT object detection for three target classes:
- **orange mallet** (class 1)
- **rock pick hammer** (class 2)
- **water bottle** (class 3)

The trained model is exported as a TorchScript file with text embeddings baked in (vision-only inference), used by the C++ runtime in `src/object-detection/`.

## Setup

```bash
conda activate hsr
pip install -r requirements.txt
```

## Files

| File | Description |
|------|-------------|
| `train.py` | Fine-tune OWL-ViT detection heads (frozen backbone) |
| `dataset.py` | COCO dataset loader for training |
| `collect_data.py` | Download web images + auto-annotate with Grounding DINO |
| `add_real_data.py` | Add real-world photos to dataset with auto-annotation |
| `capture.py` | Capture images from RealSense camera for dataset collection |

## Dataset Structure

```
datasets/web_coco/
├── orange_mallet/
│   ├── train/
│   │   ├── data/         # images (000001.jpg, ...)
│   │   └── labels.json   # COCO format annotations
│   └── val/
│       ├── data/
│       └── labels.json
├── rock_pick_hammer/
│   └── ...
└── water_bottle/
    └── ...
```

## Workflow

### 1. Collect Data

Download web images with auto-annotation:

```bash
python collect_data.py --preset orange_hammer
python collect_data.py --preset rock_pick
python collect_data.py --preset water_bottle
```

### 2. Add Real-World Photos (Optional)

Capture images with RealSense:

```bash
python capture.py
```
- **Space** = take photo, **1/2/3** = switch class, **q** = quit

Then annotate and merge into dataset:

```bash
# Preview annotations first
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet --preview

# Merge into dataset
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet
python add_real_data.py --input captured_images/rock_pick_hammer --class rock_pick_hammer
python add_real_data.py --input captured_images/water_bottle --class water_bottle
```

### 3. Train

```bash
python train.py \
  --datasets "orange mallet=datasets/web_coco/orange_mallet" \
             "rock pick hammer=datasets/web_coco/rock_pick_hammer" \
             "water bottle=datasets/web_coco/water_bottle" \
  --output runs/three_class_v5 \
  --epochs 30
```

### 4. Export to TorchScript

After training, export the model for C++ inference. The export bakes text embeddings into the model so only vision inference runs at runtime:

```bash
python export.py --checkpoint runs/three_class_v5/final --output ../object-detection/owlvit_finetune.pt
```

### 5. Test

Build and run the RealSense test:

```bash
cd ../../build
cmake --build . --target realsense_test -j$(nproc)
./object-detection/realsense_test
```

Controls: **1/2/3** = toggle class, **4** = all, **+/-** = adjust threshold, **q** = quit

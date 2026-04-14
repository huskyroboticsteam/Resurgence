# OWL-ViT Fine-tune

Fine-tune OWL-ViT for three object classes used in the HSR robot:
- **orange mallet** (class 1)
- **rock pick hammer** (class 2)
- **water bottle** (class 3)

The trained model exports as a TorchScript file (`owlvit_finetune.pt`) used by the C++ object detection node.

---

## Environment Setup

### 1. Activate conda environment

```bash
conda activate hsr
```

### 2. Install dependencies

```bash
cd /home/thomas/hsr/Resurgence/src/model-finetune
pip install -r requirements.txt
```

`requirements.txt` includes: torch, torchvision, transformers, pillow, scipy, tqdm, opencv-python, pyrealsense2.

### 3. Install Grounding DINO

Grounding DINO is used to auto-annotate photos. It is pulled automatically via `transformers` — no separate install needed. The model (`IDEA-Research/grounding-dino-base`) downloads from HuggingFace the first time you run `add_real_data.py`.

If you are not logged in to HuggingFace:

```bash
huggingface-cli login
```

Enter your token from https://huggingface.co/settings/tokens.

---

## Complete Workflow

### Step 1 — Capture real-world photos

Connect the RealSense camera, then run:

```bash
cd /home/thomas/hsr/Resurgence/src/model-finetune
python capture.py
```

Controls:
- `Space` — take a photo
- `1` — switch to orange_mallet
- `2` — switch to rock_pick_hammer
- `3` — switch to water_bottle
- `q` — quit

Photos are saved to `captured_images/<class_name>/`. Aim for **200+ photos per class** at different angles, distances, and lighting conditions.

Output folder structure after capture:
```
captured_images/
├── orange_mallet/
├── rock_pick_hammer/
└── water_bottle/
```

---

### Step 2 — Preview annotations (optional but recommended)

Before adding photos to the dataset, preview what Grounding DINO detects:

```bash
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet --preview
```

This creates a `captured_images/orange_mallet/_preview/` folder with bounding boxes drawn on each image. Open the folder and check that the boxes look correct before proceeding.

If detections are missing, lower the threshold (default is 0.25):

```bash
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet --preview --box-threshold 0.15
```

---

### Step 3 — Add photos to the dataset

This annotates the photos and merges them into `datasets/web_coco/`. 10% of the images are automatically split into the val set.

```bash
python add_real_data.py --input captured_images/orange_mallet --class orange_mallet
python add_real_data.py --input captured_images/rock_pick_hammer --class rock_pick_hammer
python add_real_data.py --input captured_images/water_bottle --class water_bottle
```

You can also point `--input` at any folder of images, not just from `capture.py`. For example if you manually downloaded images from Google:

```bash
python add_real_data.py --input ~/Downloads/orange_mallet_photos --class orange_mallet
```

---

### Alternative — Download web images automatically

Instead of taking real photos, you can download images from the web and auto-annotate them:

```bash
python collect_data.py --preset orange_mallet
python collect_data.py --preset rock_pick_hammer
python collect_data.py --preset water_bottle
```

This downloads images using search queries defined in the presets, annotates them with Grounding DINO, and saves them directly into `datasets/web_coco/`. Note: the Google image crawler may be unreliable — if it fails, download images manually and use `add_real_data.py` instead.

To download with a custom query:

```bash
python collect_data.py \
  --query "orange dead blow hammer" "orange mallet tool" \
  --prompt "orange dead blow hammer." \
  --category "Hammer" \
  --output datasets/web_coco/orange_mallet \
  --max-images 100
```

---

### Step 4 — Train

Replace `my_run` with the next version number each time you retrain:

```bash
python train.py \
  --datasets "orange mallet=datasets/web_coco/orange_mallet" \
             "rock pick hammer=datasets/web_coco/rock_pick_hammer" \
             "water bottle=datasets/web_coco/water_bottle" \
  --output runs/three_class_my_run \
  --epochs 30
```

Training prints loss each epoch:
```
epoch  1/30  train=0.24  val=0.18  ★ best
epoch  2/30  train=0.18  val=0.21
...
```

Checkpoints are saved every 5 epochs to `runs/three_class_my_run/epoch-N/`, plus `best/` and `final/`.

---

### Step 5 — Export to TorchScript

Use `final` unless `best` gives clearly better results:

```bash
python export.py \
  --checkpoint runs/three_class_my_run/final \
  --output ../object-detection/owlvit_finetune.pt
```

Output is ~342 MB. This is the file the C++ node loads at runtime.

---

### Step 6 — Test

#### Option A — Quick test with a static image (no camera needed)

```bash
cd /home/thomas/hsr/Resurgence/src/model-finetune
python -c "
import torch, glob
from PIL import Image
from transformers import OwlViTProcessor

class_names = ['no object', 'orange mallet', 'rock pick hammer', 'water bottle']
model = torch.jit.load('../object-detection/owlvit_finetune.pt', map_location='cpu')
processor = OwlViTProcessor.from_pretrained('runs/three_class_my_run/final')

img = Image.open('/path/to/your/image.jpg').convert('RGB')
pv = processor(images=img, return_tensors='pt')['pixel_values']
with torch.no_grad():
    logits, boxes = model(pv)

probs = logits.softmax(-1)
max_probs, max_cls = probs[0].max(-1)
mask = max_cls > 0
if mask.any():
    fp = max_probs.clone(); fp[~mask] = 0
    top = fp.argsort(descending=True)[:5]
    for i in top:
        cls = max_cls[i].item()
        conf = max_probs[i].item()
        if conf > 0.5:
            print(f'{class_names[cls]}: {conf:.3f}  box={[round(b,3) for b in boxes[0,i].tolist()]}')
else:
    print('No detection')
"
```

You can also run against the full val set to get accuracy numbers:

```bash
python -c "
import torch, glob
from PIL import Image
from transformers import OwlViTProcessor

class_names = ['no object', 'orange mallet', 'rock pick hammer', 'water bottle']
model = torch.jit.load('../object-detection/owlvit_finetune.pt', map_location='cpu')
processor = OwlViTProcessor.from_pretrained('runs/three_class_my_run/final')

for cls_name, folder, target in [('orange_mallet','orange_mallet',1),('rock_pick_hammer','rock_pick_hammer',2),('water_bottle','water_bottle',3)]:
    imgs = sorted(glob.glob(f'datasets/web_coco/{folder}/val/data/*'))
    correct = 0
    for img_path in imgs:
        img = Image.open(img_path).convert('RGB')
        pv = processor(images=img, return_tensors='pt')['pixel_values']
        with torch.no_grad():
            logits, _ = model(pv)
        probs = logits.softmax(-1)
        max_probs, max_cls = probs[0].max(-1)
        mask = max_cls > 0
        if mask.any():
            fp = max_probs.clone(); fp[~mask] = 0
            if max_cls[fp.argmax()].item() == target:
                correct += 1
    print(f'{cls_name}: {correct}/{len(imgs)} ({100*correct/len(imgs):.1f}%)')
"
```

#### Option B — Test with RealSense camera

Delete the old cached model so it loads the new one:

```bash
rm /home/thomas/hsr/Resurgence/src/object-detection/owlvit_finetune.pt
```

Build and run:

```bash
cd /home/thomas/hsr/Resurgence/build
cmake --build . --target realsense_test -j$(nproc)
./object-detection/realsense_test
```

Controls:
- `1` — detect orange mallet only
- `2` — detect rock pick hammer only
- `3` — detect water bottle only
- `4` — detect all classes
- `+` / `-` — raise or lower confidence threshold
- `q` — quit

---

### Step 7 — Upload to HuggingFace

After confirming the model works, upload so others can use it:

```bash
python -c "
from huggingface_hub import HfApi
HfApi().upload_file(
    path_or_fileobj='../object-detection/owlvit_finetune.pt',
    path_in_repo='owlvit_finetune.pt',
    repo_id='thomas0829/OWL-ViT_Finetune',
)
print('Done')
"
```

Others pulling the project will automatically download this file when they first run `realsense_test`.

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

---

## Files

| File | Description |
|------|-------------|
| `train.py` | Fine-tune OWL-ViT detection heads (frozen backbone) |
| `export.py` | Export trained checkpoint to TorchScript |
| `dataset.py` | COCO dataset loader |
| `capture.py` | Capture images from RealSense camera |
| `add_real_data.py` | Auto-annotate photos with Grounding DINO and merge into dataset |
| `collect_data.py` | Download web images and auto-annotate |
| `requirements.txt` | Python dependencies |

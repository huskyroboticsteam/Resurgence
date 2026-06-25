#!/usr/bin/env bash
set -euo pipefail

# ── Config ──────────────────────────────────────────────────────────────
PYTHON="${PYTHON:-/home/thomas/anaconda3/envs/hsr/bin/python}"
BASE="src/model-finetune"
RUN_NAME="${1:-three_class_v2}"

# ── Train ───────────────────────────────────────────────────────────────
echo "============================================================"
echo "Training: $RUN_NAME"
echo "============================================================"

"$PYTHON" "$BASE/train.py" \
    --datasets \
        "orange mallet=$BASE/datasets/web_coco/orange_hammer" \
        "rock pick hammer=$BASE/datasets/web_coco/rock_pick" \
        "water bottle=$BASE/datasets/web_coco/water_bottle" \
    --output "$BASE/runs/$RUN_NAME" \
    --epochs 30 \
    --batch-size 4 \
    --lr 1e-4

# ── Export ──────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "Exporting TorchScript"
echo "============================================================"

"$PYTHON" "$BASE/export.py" \
    --model-dir "$BASE/runs/$RUN_NAME/best" \
    --output "$BASE/exports/$RUN_NAME.pt"

echo ""
echo "============================================================"
echo "Done!"
echo "  Model:   $BASE/exports/$RUN_NAME.pt"
echo "  Tokens:  $BASE/exports/$RUN_NAME.tokens.json"
echo "  Classes: no object, orange mallet, rock pick hammer, water bottle"
echo "============================================================"

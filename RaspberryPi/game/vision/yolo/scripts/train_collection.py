#!/usr/bin/env python3
"""Fine-tune YOLO11n-seg on the reviewed collection_v1 dataset."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / 'data/collection/collection_v1/dataset.yaml'
BASE_WEIGHTS = ROOT / 'models/yolo11n-seg.pt'
RUNS = ROOT / 'runs/collection_v1'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frozen-epochs', type=int, default=8)
    parser.add_argument('--finetune-epochs', type=int, default=50)
    parser.add_argument('--batch', type=int, default=16)
    args = parser.parse_args()
    if not DATASET.exists() or not BASE_WEIGHTS.exists():
        raise FileNotFoundError('reviewed dataset or official base weights are missing')
    if not torch.cuda.is_available():
        raise RuntimeError('GPU training requires a CUDA-enabled PyTorch installation')
    if (RUNS / 'stage1_frozen').exists() or (RUNS / 'stage2_finetune').exists():
        raise FileExistsError('training run already exists; inspect it before rerunning')

    common = dict(data=str(DATASET), imgsz=320, batch=args.batch, device=0,
                  workers=0, project=str(RUNS), seed=2026,
                  deterministic=True, optimizer='AdamW', weight_decay=0.0005,
                  hsv_h=0.0, hsv_s=0.15, hsv_v=0.15,
                  degrees=3.0, translate=0.05, scale=0.15,
                  perspective=0.0, mosaic=0.0, mixup=0.0,
                  copy_paste=0.0, erasing=0.0, plots=True)
    stage1 = YOLO(str(BASE_WEIGHTS))
    stage1.train(name='stage1_frozen', epochs=args.frozen_epochs,
                 freeze=10, lr0=0.001, lrf=0.2,
                 patience=args.frozen_epochs, **common)
    stage1_best = RUNS / 'stage1_frozen/weights/best.pt'
    if not stage1_best.exists():
        raise FileNotFoundError(stage1_best)

    stage2 = YOLO(str(stage1_best))
    stage2.train(name='stage2_finetune', epochs=args.finetune_epochs,
                 freeze=0, lr0=0.0002, lrf=0.1,
                 patience=12, **common)
    best = RUNS / 'stage2_finetune/weights/best.pt'
    if not best.exists():
        raise FileNotFoundError(best)
    print(f'Frozen-stage checkpoint: {stage1_best}')
    print(f'Fine-tune checkpoint: {best}')
    print('Select between them using validation masks and instance counts before test evaluation.')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Fine-tune YOLO11n-pose on six reviewed visible cube outline points."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / 'data/collection/visible_outline_v2/dataset.yaml'
PRETRAINED = ROOT / 'models/yolo11n-pose.pt'
RUNS = ROOT / 'runs/visible_outline_v2'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frozen-epochs', type=int, default=8)
    parser.add_argument('--finetune-epochs', type=int, default=50)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--runs', type=Path, default=RUNS)
    args = parser.parse_args()
    if args.imgsz < 160 or args.imgsz % 32:
        parser.error('--imgsz must be at least 160 and divisible by 32')
    if not DATASET.exists() or not PRETRAINED.exists():
        raise FileNotFoundError('reviewed visible-outline data or base weights missing')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA-enabled PyTorch is required for this training run')
    if args.runs.exists():
        raise FileExistsError(f'training output already exists: {args.runs}')

    common = dict(data=str(DATASET), imgsz=args.imgsz, batch=args.batch,
                  device=0, workers=0, project=str(args.runs.resolve()), seed=2026,
                  deterministic=True, optimizer='AdamW', weight_decay=0.0005,
                  hsv_h=0.0, hsv_s=0.15, hsv_v=0.15,
                  degrees=3.0, translate=0.05, scale=0.15,
                  perspective=0.0, mosaic=0.0, mixup=0.0,
                  copy_paste=0.0, erasing=0.0, fliplr=0.0, plots=True)
    frozen = YOLO(str(PRETRAINED))
    frozen.train(name='stage1_frozen', epochs=args.frozen_epochs,
                 freeze=10, lr0=0.001, lrf=0.2,
                 patience=args.frozen_epochs, **common)
    first_best = args.runs / 'stage1_frozen/weights/best.pt'
    if not first_best.exists():
        raise FileNotFoundError(first_best)
    finetune = YOLO(str(first_best))
    finetune.train(name='stage2_finetune', epochs=args.finetune_epochs,
                   freeze=0, lr0=0.0002, lrf=0.1,
                   patience=12, **common)
    second_best = args.runs / 'stage2_finetune/weights/best.pt'
    if not second_best.exists():
        raise FileNotFoundError(second_best)
    print(f'Frozen checkpoint: {first_best}')
    print(f'Fine-tuned checkpoint: {second_best}')
    print('Select the model only after reviewing validation overlays and edge errors.')


if __name__ == '__main__':
    main()

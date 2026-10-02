"""Backward-compatible import and CLI entry for the OpenCV backend."""
if __name__ == '__main__':
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from vision.opencv.camera_tuner import main
    main()
else:
    from .opencv.camera_tuner import *

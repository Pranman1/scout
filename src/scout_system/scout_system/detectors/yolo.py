"""YOLO detector backend -- stub.

Wire up ultralytics / torch weights here when ready. The intent is that
``build_detector('yolo', cfg)`` is a drop-in replacement for the HSV
backend, so nothing downstream has to change.

Expected future config::

    detector:
      backend: yolo
      yolo:
        weights: /abs/path/to/scout_hazards.pt
        conf_threshold: 0.4
        iou_threshold: 0.45
        class_to_label:      # model class idx -> color/label key
          0: red
          1: yellow
          2: blue

Until this is implemented, instantiating the class raises so the node
fails fast instead of silently publishing zero detections.
"""
from __future__ import annotations

from typing import List

import numpy as np

from .base import Detection, Detector


class YoloDetector(Detector):
    def __init__(self, config: dict):  # noqa: D401 - stub
        raise NotImplementedError(
            'YOLO backend is not implemented yet. Use detector.backend: hsv '
            "or fill in scout_system/detectors/yolo.py."
        )

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:  # pragma: no cover
        return []

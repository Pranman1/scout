"""HSV color-threshold hazard detector (SKELETON).

Config block expected in ``hazard_params.yaml``::

    detector:
      backend: hsv
      min_area_px: 400
      hsv:
        red:
          - {lo: [  0, 120,  70], hi: [ 10, 255, 255]}
          - {lo: [170, 120,  70], hi: [180, 255, 255]}
        yellow:
          - {lo: [ 20, 120, 120], hi: [ 35, 255, 255]}
        blue:
          - {lo: [100, 150,   0], hi: [130, 255, 255]}

Red gets two ranges because its hue wraps around 0 in OpenCV's 0-180
HSV convention. Every color value is a *list of* ``{lo, hi}`` dicts,
even if length 1 -- keeps the code path uniform.

You implement ``detect``. The config-parsing part of ``__init__`` is
already done because it's just dict/numpy plumbing that doesn't teach
you anything about CV.
"""
from __future__ import annotations

from typing import Dict, List

import cv2
import numpy as np

from .base import Detection, Detector


class HSVDetector(Detector):
    def __init__(self, config: dict):
        cfg = config or {}
        self.min_area = int(cfg.get('min_area_px', 400))
        raw: Dict[str, list] = cfg.get('hsv', {})

        self.ranges: Dict[str, List[tuple]] = {}
        for label, spans in raw.items():
            prepared = []
            for span in spans:
                lo = np.array(span['lo'], dtype=np.uint8)
                hi = np.array(span['hi'], dtype=np.uint8)
                prepared.append((lo, hi))
            self.ranges[label] = prepared

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        """Segment each configured color and return one Detection per blob.

        TODO(you):
            1. Guard: return [] if ``image_bgr`` is None or empty.
            2. Convert BGR -> HSV once (``cv2.cvtColor(..., COLOR_BGR2HSV)``).
            3. For each label in ``self.ranges``:
                 a. OR together a mask over every (lo, hi) span with
                    ``cv2.inRange`` -- this handles red's hue wrap.
                 b. Clean speckle with a small open + close
                    (``cv2.morphologyEx`` with a 3x3 kernel; ~1 open,
                    ~2 close iterations is a good starting point).
                 c. ``cv2.findContours(mask, RETR_EXTERNAL,
                    CHAIN_APPROX_SIMPLE)``.
                 d. For each contour: skip if ``cv2.contourArea < self.min_area``.
                    Otherwise grab ``cv2.boundingRect`` and emit a
                    ``Detection`` with the box center (cx, cy), width,
                    height, label, and a confidence in [0, 1].
                    Simple confidence heuristic: saturate at 4 * min_area,
                    i.e. ``min(1.0, area / (4.0 * self.min_area))``.
            4. Return the accumulated list.

        Testing tip: this class is pure OpenCV, so write a tiny
        ``if __name__ == '__main__':`` harness that feeds it a
        ``cv2.imread`` of a screenshot from Gazebo. No ROS needed.
        """
        raise NotImplementedError("TODO(you): implement HSV threshold + contour detection.")

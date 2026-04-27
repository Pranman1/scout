"""HSV color-threshold hazard detector.

Config block expected in hazard_params.yaml::

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

Red gets two ranges because its hue wraps around 0 in OpenCV's 0-180 HSV.
Every color value must be a *list of* {lo, hi} dicts even if length 1.
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
        if image_bgr is None or image_bgr.size == 0:
            return []

        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        out: List[Detection] = []

        for label, spans in self.ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in spans:
                mask |= cv2.inRange(hsv, lo, hi)

            # Small open + close to kill speckle without shrinking real cubes.
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                # Confidence: saturate at 4x min area. Enough signal for now;
                # a proper detector would use a learned score.
                conf = float(min(1.0, area / (4.0 * self.min_area)))
                out.append(
                    Detection(
                        cx=x + w / 2.0,
                        cy=y + h / 2.0,
                        w=float(w),
                        h=float(h),
                        label=label,
                        confidence=conf,
                    )
                )
        return out

"""
hazard_params.yaml:

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

ther initial ranges we are suing as rbakcet for the HSV valeus 
"""

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
        """
        if image_bgr is None or image_bgr.size == 0:
            return []

        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        detections = []

        for label, spans in self.ranges.items():
            mask = np.zeros(image_bgr.shape[:2], dtype=np.uint8)
            for lo, hi in spans:
                mask2 = cv2.inRange(hsv, lo, hi)
                cv2.bitwise_or(mask, mask2, mask)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=2)

            countours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in countours:
              area = cv2.contourArea(contour)
              if area < self.min_area:
                continue
              x, y, w, h = cv2.boundingRect(contour)

              cx = x + w/2.0 
              cy = y + h/2.0

              confidence = min(1.0, area / (4.0 * self.min_area))
              # basically sclaing up tot satyration of 1.0 based on the area of the contour
              detections.append(Detection(cx, cy, w, h, label, confidence))
              
         
        return detections
                
"""Detector interface shared by every backend."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class Detection:
    """One bounding-box observation in image space.

    Coordinates are pixels in the input image, with origin at the top-left
    (OpenCV convention). ``label`` is the color/class key configured in
    hazard_params.yaml (e.g. "red"); the hazard_detector node maps it to a
    semantic category before publishing.
    """

    cx: float           # bbox center, x (pixels)
    cy: float           # bbox center, y (pixels)
    w: float            # bbox width  (pixels)
    h: float            # bbox height (pixels)
    label: str          # color / class key
    confidence: float   # [0.0, 1.0]


class Detector:
    """Abstract base; subclasses implement :meth:`detect`."""

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:  # pragma: no cover
        raise NotImplementedError
